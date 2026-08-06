#!/usr/bin/env python3
"""Banc de mesure de MPMbox, construit a partir de Examples/helloWorld.

    python3 Tests/benchmark.py                    # mesure et compare a la reference
    python3 Tests/benchmark.py --save             # (re)genere la reference
    python3 Tests/benchmark.py -n 3               # 3 repetitions, on garde la plus rapide
    python3 Tests/benchmark.py -k bspline         # un seul cas
    python3 Tests/benchmark.py --threads 1,2,4,8  # passage a l'echelle OpenMP
    python3 Tests/benchmark.py --exe BUILD/mpmbox

Le banc mesure DEUX choses, et les deux comptent :

  - le temps, global et par phase du pas de temps, lu dans le fichier
    'perflog_mpmbox_<n>.txt' que le profileur integre au code produit ;
  - le RESULTAT, resume par quelques scalaires. Une optimisation qui change le
    resultat n'est pas une optimisation, c'est une regression. Le banc affiche
    l'ecart relatif, et signale tout ce qui depasse 1e-9.

Le pas de temps est fixe explicitement, en-dessous de la limite CFL, pour que
convergenceConditions ne l'ajuste pas : le nombre de pas doit rester le meme
d'une version a l'autre, sinon les temps ne sont pas comparables.

La reference est rangee dans Tests/benchmark_ref.json. La comparer entre deux
machines n'a pas de sens ; elle sert a mesurer l'effet d'une modification du
code, a machine constante.
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mpmconf import Conf  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WORK = os.path.join(HERE, "_bench")
REF = os.path.join(HERE, "benchmark_ref.json")

# ---------------------------------------------------------------------------
# Les cas
# ---------------------------------------------------------------------------

# Reprise de Examples/helloWorld : un bloc de Mohr-Coulomb lache dans une boite
# ouverte faite de trois parois. Seuls le pas de temps, la duree, la fonction de
# forme et la taille des points changent d'un cas a l'autre.
TEMPLATE = """\
oneStepType {scheme}
result_folder .
tolmass 1.e-5
gravity 0.0  -9.81
finalTime {tmax}
confPeriod {confPeriod}
proxPeriod 10
dt {dt}
t 0.0
ShapeFunction {sf}
splitting 0
enablePIC 0.01

model MohrCoulomb  MC  1.0e9  0.42  0.3  1e2  0.01

set_node_grid  W.H.lx.ly  0.4  0.3  {cell} {cell}

set_MP_grid  0  MC  2700.0  {x0}  {y0}  {x1}  {y1}  {mpsize}

Obstacle Line    1      0.36  0.26  0.36  0.04  freeze
Obstacle Line    1      0.36  0.04  0.04  0.04  freeze
Obstacle Line    1      0.04  0.04  0.04  0.26  freeze

BoundaryForceLaw frictionalViscoElastic 1

set  mu        0  1  0.4
set  kn        0  1  1e7
set  kt        0  1  1e7
set  viscRate  0  1  0.98
"""

# dt fixe sous la limite CFL de ce materiau (E = 1 GPa, rho = 2700), pour que
# convergenceConditions ne le rabaisse pas et que le nombre de pas soit stable.
DT = 9.0e-7

CASES = [
    dict(name="bspline", nsteps=11000, sf="BSpline", cell=0.01, mpsize=0.005,
         scheme="ModifiedLagrangian",
         doc="Le cas helloWorld tel quel. 384 points, 16 noeuds par element : "
             "les fonctions de forme dominent tout le reste."),
    dict(name="linear", nsteps=11000, sf="Linear", cell=0.01, mpsize=0.005,
         scheme="ModifiedLagrangian",
         doc="Meme cas avec 4 noeuds par element. Le cout bascule sur la loi de "
             "comportement et la liste des noeuds vivants."),
    dict(name="dense", nsteps=3000, sf="Linear", cell=0.01, mpsize=0.0025,
         scheme="ModifiedLagrangian",
         doc="1536 points sur la meme grille, 16 par maille. Montre comment le "
             "cout suit le nombre de points a grille constante."),
    dict(name="usf", nsteps=11000, sf="Linear", cell=0.01, mpsize=0.005,
         scheme="UpdateStressFirst",
         doc="Un autre schema d'integration, pour verifier qu'une optimisation "
             "faite dans les parties communes profite bien aux trois."),
    dict(name="bigspline", nsteps=150, sf="BSpline", cell=0.0025, mpsize=0.00125,
         scheme="ModifiedLagrangian", dt=2.0e-7,
         x0=0.05, y0=0.05, x1=0.30, y1=0.25, heavy=True,
         doc="Le cas de reference a l'echelle : environ 32000 points ET des "
             "B-splines, c'est-a-dire ce qui est reellement utilise. Exclu par "
             "defaut (-k bigspline)."),
    dict(name="big", nsteps=300, sf="Linear", cell=0.0025, mpsize=0.00125,
         scheme="ModifiedLagrangian", dt=2.0e-7,
         x0=0.05, y0=0.05, x1=0.30, y1=0.25, heavy=True,
         doc="Environ 32000 points, 4 par maille. Sert a mesurer le passage a "
             "l'echelle OpenMP : les cas ci-dessus sont trop petits pour que "
             "le parallelisme ait une chance. Exclu par defaut (-k big)."),
]

# Phases suivies dans le profil. Les noms viennent des START_TIMER du code.
PHASES = ["shape functions", "live node list", "P2G mass momentum", "P2G internal forces",
          "contact forces", "nodal update", "updateMPVelocity", "P2G velocity remap",
          "updateTransformationGradient", "updateStrainAndStress", "G2P position",
          "MP volume corners", "grid reset"]


# ---------------------------------------------------------------------------
# Execution
# ---------------------------------------------------------------------------

def buildInput(case):
    # finalTime a mi-chemin du pas suivant : le dernier conf-file est ecrit a
    # coup sur (voir D11 dans Doc/BUGS.md)
    dt = case.get("dt", DT)
    return TEMPLATE.format(scheme=case["scheme"], sf=case["sf"], dt=repr(dt),
                           tmax=repr((case["nsteps"] + 0.5) * dt),
                           confPeriod=case["nsteps"], cell=case["cell"],
                           mpsize=case["mpsize"],
                           x0=case.get("x0", 0.04), y0=case.get("y0", 0.04),
                           x1=case.get("x1", 0.1), y1=case.get("y1", 0.20))


def readProfile(path):
    """Lit perflog_mpmbox_<n>.txt : {nom: secondes}.

    Les durees negatives ou nulles sont ecartees : le profileur en a produit une
    une fois, et un temps aberrant se lit ensuite comme une acceleration
    spectaculaire. Mieux vaut une phase absente du rapport qu'un chiffre faux.
    """
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path):
        m = re.match(r"\s*(\S.*?) (\d+) ([\d.eE+-]+)\s*$", line.rstrip())
        if m:
            t = float(m.group(3))
            if t > 0.0:
                out[m.group(1)] = t
    return out


def fingerprint(conf):
    """Quelques scalaires qui resument l'etat final du calcul."""
    n = max(1, len(conf.MP))
    cx, cy = conf.centerOfMass()
    return {
        "nbMP": len(conf.MP),
        "t": conf.t,
        "comX": cx,
        "comY": cy,
        "kinetic": sum(0.5 * p.mass * (p.vx ** 2 + p.vy ** 2) for p in conf.MP),
        "meanSyy": sum(p.stress[3] for p in conf.MP) / n,
        "meanSxx": sum(p.stress[0] for p in conf.MP) / n,
        "maxAbsStress": conf.maxAbsStress(),
    }


def runCase(exe, case, threads, repeats):
    d = os.path.join(WORK, case["name"] + ("_j%d" % threads if threads != 1 else ""))
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    with open(os.path.join(d, "input.txt"), "w") as f:
        f.write(buildInput(case))

    best = None
    prof = {}
    for _ in range(repeats):
        for junk in os.listdir(d):
            if junk.startswith("conf") or junk.startswith("perflog"):
                os.remove(os.path.join(d, junk))
        t0 = time.time()
        p = subprocess.run([exe, "input.txt", "-v", "1", "-j", str(threads)], cwd=d,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        el = time.time() - t0
        if p.returncode != 0:
            raise RuntimeError("%s : mpmbox a retourne %d\n%s"
                               % (case["name"], p.returncode,
                                  p.stdout.decode("utf-8", "replace")[-800:]))
        # On garde la plus rapide : le bruit d'une machine partagee ne fait
        # qu'ajouter du temps, jamais en retirer.
        if best is None or el < best:
            best = el
            for f in os.listdir(d):
                if f.startswith("perflog"):
                    prof = readProfile(os.path.join(d, f))

    conf = Conf(os.path.join(d, "conf1.txt"))
    return dict(wall=best, profile=prof, result=fingerprint(conf), dir=d)


# ---------------------------------------------------------------------------
# Affichage
# ---------------------------------------------------------------------------

C = {"g": "\033[32m", "r": "\033[31m", "y": "\033[33m", "d": "\033[2m",
     "b": "\033[1m", "o": "\033[0m"}
if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
    C = {k: "" for k in C}


def speedup(ref, now):
    if not ref or not now:
        return ""
    r = ref / now
    col = C["g"] if r > 1.02 else (C["r"] if r < 0.98 else C["d"])
    return "  %s%5.2fx%s" % (col, r, C["o"])


def compareResults(name, ref, now):
    """Retourne la liste des ecarts significatifs sur les scalaires du resultat."""
    bad = []
    for k, v in now.items():
        if k not in ref:
            continue
        a = ref[k]
        scale = max(abs(a), abs(v))
        if scale < 1e-30:
            continue
        rel = abs(v - a) / scale
        if rel > 1e-9:
            bad.append((k, a, v, rel))
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-k", metavar="MOTIF", help="ne garder que les cas dont le nom contient MOTIF")
    # Trois repetitions par defaut : une seule mesure sur une machine partagee
    # se trompe couramment de 40 %. C'est la plus rapide qui est retenue, le
    # bruit ne pouvant qu'ajouter du temps.
    ap.add_argument("-n", type=int, default=3, metavar="N", help="repetitions (defaut 3)")
    ap.add_argument("--threads", default="1", help="liste de nombres de fils, ex. 1,2,4")
    ap.add_argument("--save", action="store_true", help="ecrire la reference et sortir")
    ap.add_argument("--exe", help="chemin de l'executable mpmbox")
    ap.add_argument("--phases", action="store_true", help="detail par phase du pas de temps")
    args = ap.parse_args()

    exe = args.exe or os.environ.get("MPMBOX")
    if not exe:
        for cand in ("BUILD/mpmbox", "build/mpmbox", "bld/mpmbox"):
            p = os.path.join(ROOT, cand)
            if os.path.isfile(p) and os.access(p, os.X_OK):
                exe = p
                break
    if not exe or not os.path.isfile(exe):
        print("Executable mpmbox introuvable (--exe, ou MPMBOX).", file=sys.stderr)
        return 2
    exe = os.path.abspath(exe)

    # Les cas lourds ne sont pris que s'ils sont demandes nommement : ils
    # pesent plus que tous les autres reunis.
    if args.k:
        cases = [c for c in CASES if args.k in c["name"]]
    else:
        cases = [c for c in CASES if not c.get("heavy", False)]
    threads = [int(t) for t in args.threads.split(",")]
    os.makedirs(WORK, exist_ok=True)

    ref = {}
    if os.path.exists(REF) and not args.save:
        ref = json.load(open(REF))

    print("%smpmbox%s : %s" % (C["b"], C["o"], exe))
    if ref:
        print("%sreference%s : %s  (%s)" % (C["b"], C["o"], REF, ref.get("_when", "?")))
    print()

    out = {"_when": time.strftime("%Y-%m-%d %H:%M"), "_exe": exe}
    regressions = []

    for nt in threads:
        if len(threads) > 1:
            print("%s--- %d fil(s) ---%s" % (C["b"], nt, C["o"]))
        head = "  %-10s %10s %10s %12s" % ("cas", "temps", "us/pas", "us/pas/MP")
        print(head + ("  gain" if ref else ""))
        for case in cases:
            key = "%s_j%d" % (case["name"], nt)
            r = runCase(exe, case, nt, args.n)
            us = 1e6 * r["wall"] / case["nsteps"]
            usmp = us / r["result"]["nbMP"]
            prev = ref.get(key, {})
            line = "  %-10s %9.2fs %10.1f %12.4f" % (case["name"], r["wall"], us, usmp)
            print(line + speedup(prev.get("wall"), r["wall"]))

            if prev.get("result"):
                bad = compareResults(case["name"], prev["result"], r["result"])
                if bad:
                    regressions.append((case["name"], bad))

            if args.phases:
                step = next((v for k, v in r["profile"].items() if k.endswith("step")), 0.0)
                for ph in PHASES:
                    if ph in r["profile"] and step > 0:
                        t = r["profile"][ph]
                        pr = prev.get("profile", {}).get(ph)
                        print("       %-30s %8.3fs %5.1f %%%s"
                              % (ph, t, 100.0 * t / step, speedup(pr, t)))

            out[key] = dict(wall=r["wall"], profile=r["profile"], result=r["result"],
                            nsteps=case["nsteps"])
        print()

    if regressions:
        print("%sLE RESULTAT A CHANGE%s -- une optimisation ne doit pas modifier "
              "la physique :" % (C["r"], C["o"]))
        for name, bad in regressions:
            print("  %s :" % name)
            for k, a, v, rel in bad:
                print("      %-14s %.12g -> %.12g   (ecart relatif %.2e)" % (k, a, v, rel))
        print()

    if args.save:
        json.dump(out, open(REF, "w"), indent=1)
        print("Reference ecrite dans %s" % REF)
    elif not ref:
        print("%sPas de reference%s : lancer 'python3 Tests/benchmark.py --save' "
              "AVANT de modifier le code." % (C["y"], C["o"]))

    return 1 if regressions else 0


if __name__ == "__main__":
    sys.exit(main())
