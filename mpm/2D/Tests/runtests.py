#!/usr/bin/env python3
"""Tests de non-regression de MPMbox (mpm/2D).

    python3 Tests/runtests.py                 # tout
    python3 Tests/runtests.py T01 T10         # une selection
    python3 Tests/runtests.py -k freefall     # par motif
    python3 Tests/runtests.py -l              # lister sans executer
    python3 Tests/runtests.py --exe path/to/mpmbox

Trois familles de tests :

  INVARIANT  une propriete physique ou numerique qui doit etre vraie
             AVANT et APRES les corrections. Un echec est une regression.

  XFAIL      un defaut connu, decrit dans Doc/BUGS.md sous l'identifiant
             indique. Le test doit echouer aujourd'hui et passer une fois le
             defaut corrige. Un XPASS signale donc une bonne nouvelle : il
             faut alors basculer le test en INVARIANT.

  ROBUST     le code doit refuser proprement une entree invalide (code de
             retour non nul, pas de signal). Ce sont des XFAIL tant que les
             defauts de la classe A ne sont pas corriges.

Aucune dependance hors bibliotheque standard.
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mpmconf import Conf, maxDiff, readColumns  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WORK = os.path.join(HERE, "_work")

INVARIANT, XFAIL, ROBUST = "INVARIANT", "XFAIL", "ROBUST"

# ---------------------------------------------------------------------------
# Fragments d'entree reutilises
# ---------------------------------------------------------------------------

# Grille volontairement NON carree (lx != ly) : c'est ce qui distingue une
# fonction de forme correcte d'une fonction de forme qui confond les deux.
GRID = "set_node_grid Nx.Ny.lx.ly 20 20 0.02 0.01"

HEADER = """\
oneStepType {scheme}
result_folder .
tolmass 1.e-5
gravity 0.0 {g}
finalTime {tmax}
confPeriod {confPeriod}
proxPeriod 10
dt {dt}
t 0.0
ShapeFunction {sf}
splitting 0
{pic}
"""


def header(scheme="ModifiedLagrangian", g=-10.0, tmax=0.025, dt=1e-5,
           confPeriod=2000, sf="Linear", pic="disablePIC"):
    return HEADER.format(scheme=scheme, g=g, tmax=tmax, dt=dt,
                         confPeriod=confPeriod, sf=sf, pic=pic)


def timedHeader(dt, nsteps, **kw):
    """En-tete calibree pour que conf1 soit ecrit EXACTEMENT au pas `nsteps`.

    finalTime est place a mi-chemin entre le pas nsteps et le suivant : la
    boucle execute donc les pas 0 a nsteps inclus, la sauvegarde a lieu au
    debut du pas nsteps, et le cumul t += dt ne peut pas faire manquer le
    dernier conf-file (voir D11). Indispensable pour comparer deux calculs
    menes avec des pas de temps differents.
    """
    kw.pop("tmax", None)
    kw.pop("confPeriod", None)
    return header(dt=dt, tmax=(nsteps + 0.5) * dt, confPeriod=nsteps, **kw)


def displacements(c0, c1):
    return [(b.x - a.x, b.y - a.y) for a, b in zip(c0.MP, c1.MP)]


HOOKE = "model HookeElasticity HK 1.0e7 0.3\n"

# Bloc libre, sans obstacle : 64 MP de 5 mm dans une grille de 20x20 mailles.
BLOCK = "set_MP_grid 0 HK 2000.0 0.06 0.06 0.10 0.10 0.005\n"

# Colonne posee sur la rangee de noeuds du bas, bloquee en x et en y.
COLUMN = ("set_MP_grid 0 HK 2000.0 0.06 0.0 0.10 0.06 0.005\n"
          "set_BC_line 0 0 20 1 1\n")


# ---------------------------------------------------------------------------
# Infrastructure
# ---------------------------------------------------------------------------

class Fail(Exception):
    pass


class Case:
    """Un repertoire de travail dans lequel on ecrit une entree et on lance mpmbox."""

    def __init__(self, ctx, name):
        self.ctx = ctx
        self.dir = os.path.join(WORK, name)
        shutil.rmtree(self.dir, ignore_errors=True)
        os.makedirs(self.dir)
        self.last = None

    def write(self, name, content):
        path = os.path.join(self.dir, name)
        with open(path, "w") as f:
            f.write(content)
        return path

    def run(self, inputName="input.txt", timeout=120, expectFailure=False):
        cmd = [self.ctx.exe, inputName, "-v", "1"]
        t0 = time.time()
        try:
            p = subprocess.run(cmd, cwd=self.dir, timeout=timeout,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            rc, out = p.returncode, p.stdout.decode("utf-8", "replace")
        except subprocess.TimeoutExpired:
            rc, out = None, "*** timeout apres %d s ***" % timeout
        self.ctx.elapsed += time.time() - t0
        self.last = (rc, out)
        # mpmbox installe un gestionnaire de signal qui imprime une pile d'appels
        # puis sort avec le code 0 : un plantage passe donc pour un succes aux
        # yeux de tout script. On le detecte sur la sortie.
        if CRASH_MARKER in out:
            crash = out[out.index(CRASH_MARKER):]
            raise Fail("mpmbox a plante (code de retour %s, masque par le "
                       "gestionnaire de signal)\n%s" % (rc, tail(crash, 10)))
        if not expectFailure and rc != 0:
            raise Fail("mpmbox a retourne %s\n%s" % (describeRC(rc), tail(out)))
        return rc, out

    def conf(self, num):
        path = os.path.join(self.dir, "conf%d.txt" % num)
        if not os.path.exists(path):
            raise Fail("conf%d.txt n'a pas ete produit (fichiers : %s)"
                       % (num, ", ".join(sorted(os.listdir(self.dir)))))
        return Conf(path)

    def lastConf(self):
        nums = []
        for f in os.listdir(self.dir):
            if f.startswith("conf") and f.endswith(".txt"):
                try:
                    nums.append(int(f[4:-4]))
                except ValueError:
                    pass
        if not nums:
            raise Fail("aucun conf-file produit")
        return self.conf(max(nums))

    def spy(self, name):
        path = os.path.join(self.dir, name)
        if not os.path.exists(path):
            raise Fail("le fichier de spy '%s' n'a pas ete produit" % name)
        return readColumns(path)


def describeRC(rc):
    if rc is None:
        return "timeout"
    if rc < 0:
        import signal
        try:
            return "le signal %s" % signal.Signals(-rc).name
        except ValueError:
            return "le signal %d" % (-rc)
    return "le code %d" % rc


BANNER = ("_/", "The current local time", "OpenMP", "No multithreading")

# Imprime par StackTracer quand un signal est intercepte.
CRASH_MARKER = "The Simulation received the following signal"


def tail(text, n=8):
    """Dernieres lignes utiles de la sortie : on ecarte la banniere ASCII."""
    lines = [l for l in text.splitlines()
             if l.strip() and not any(b in l for b in BANNER)]
    return "\n".join("    | " + l for l in lines[-n:]) or "    | (aucune sortie)"


class Ctx:
    def __init__(self, exe):
        self.exe = exe
        self.elapsed = 0.0
        self.notes = []

    def case(self, name):
        return Case(self, name)

    def note(self, msg):
        self.notes.append(msg)

    # -- assertions ---------------------------------------------------------

    def expect(self, cond, msg):
        if not cond:
            raise Fail(msg)

    def close(self, got, want, tol, what, rel=True):
        scale = abs(want) if (rel and want != 0.0) else 1.0
        err = abs(got - want)
        if err > tol * scale:
            raise Fail("%s : obtenu %.12g, attendu %.12g "
                       "(ecart %s %.3g > %.3g)"
                       % (what, got, want, "relatif" if rel and want else "absolu",
                          err / scale, tol))

    def below(self, got, tol, what):
        if not (abs(got) <= tol):
            raise Fail("%s : %.6g (limite %.3g)" % (what, got, tol))


TESTS = []


def test(tid, name, kind, bug=None, doc=""):
    def deco(fn):
        TESTS.append(dict(id=tid, name=name, kind=kind, bug=bug,
                          doc=doc.strip(), fn=fn))
        return fn
    return deco


def noNaN(ctx, case, label):
    """Verification systematique : aucune valeur non finie dans les conf-files."""
    for f in sorted(os.listdir(case.dir)):
        if f.startswith("conf") and f.endswith(".txt"):
            bad = Conf(os.path.join(case.dir, f)).hasNonFinite()
            if bad:
                raise Fail("%s : %s contient %d valeur(s) non finie(s), "
                           "p.ex. MP %d champ '%s'"
                           % (label, f, len(bad), bad[0][0], bad[0][1]))


# ===========================================================================
# 1. Invariants physiques  (doivent passer avant ET apres les corrections)
# ===========================================================================

def freefallCase(ctx, name, sf):
    c = ctx.case(name)
    c.write("input.txt", header(sf=sf) + HOOKE + GRID + "\n" + BLOCK)
    c.run()
    return c


def checkFreefall(ctx, c, label):
    c0, c1 = c.conf(0), c.conf(1)
    g = c1.gravity[1]
    t = c1.t

    ctx.expect(len(c1.MP) == 64, "%s : %d MP au lieu de 64" % (label, len(c1.MP)))

    # 1. Vitesse : chute libre exacte. Ne tient que si sum(N) == 1.
    for p in c1.MP:
        ctx.close(p.vy, g * t, 1e-9, "%s : v_y du MP %d" % (label, p.nb))
        ctx.below(p.vx, 1e-12, "%s : v_x du MP %d" % (label, p.nb))

    # 2. Mouvement de corps rigide : tous les MP ont la meme vitesse.
    spread = max(p.vy for p in c1.MP) - min(p.vy for p in c1.MP)
    ctx.below(spread, 1e-12, "%s : dispersion des v_y" % label)

    # 3. Position : y = y0 + g t^2/2 a l'ordre du schema (erreur ~ dt/t).
    y0 = c0.centerOfMass()[1]
    y1 = c1.centerOfMass()[1]
    ctx.close(y1 - y0, 0.5 * g * t * t, 2e-2, "%s : chute du centre de masse" % label)

    # 4. Aucune deformation, donc aucune contrainte. Ne tient que si
    #    sum(gradN) == 0.
    ctx.below(c1.maxAbsStress(), 1e-6, "%s : contrainte maximale" % label)

    # 5. Le gradient de transformation reste l'identite.
    for p in c1.MP:
        ctx.close(p.detF(), 1.0, 1e-12, "%s : det(F) du MP %d" % (label, p.nb))

    noNaN(ctx, c, label)


@test("T01", "freefall_linear", INVARIANT, doc="""
Chute libre d'un bloc de 64 points, fonction de forme Linear, mailles non
carrees. Verifie a la fois la partition de l'unite (sum N = 1, sinon
l'acceleration n'est pas g), la nullite de sum(gradN) (sinon des contraintes
parasites apparaissent) et l'appariement noeud/fonction.""")
def t01(ctx):
    checkFreefall(ctx, freefallCase(ctx, "T01_freefall_linear", "Linear"), "Linear")


@test("T02", "freefall_regularquadlinear", INVARIANT, doc="""
Identique a T01 avec RegularQuadLinear.""")
def t02(ctx):
    checkFreefall(ctx, freefallCase(ctx, "T02_freefall_rql", "RegularQuadLinear"),
                  "RegularQuadLinear")


@test("T03", "shapefunctions_equivalentes", INVARIANT, doc="""
Linear et RegularQuadLinear implementent la meme interpolation bilineaire :
sur des mailles non carrees, les deux doivent donner le meme resultat a
l'arrondi pres. C'est le test qui a valide la reecriture de Linear.
Les tolerances sont donnees champ par champ, a l'echelle de la grandeur :
comparer une contrainte nulle a 1e-14 pres n'a pas de sens.""")
def t03(ctx):
    a = freefallCase(ctx, "T03a_linear", "Linear").conf(1)
    b = freefallCase(ctx, "T03b_rql", "RegularQuadLinear").conf(1)
    ctx.expect(a.t > 0.0, "comparaison faite a t = 0")
    # position ~ 0.1 m, vitesse ~ 0.2 m/s, F ~ 1, module d'Young 1e7 Pa
    for field, tol in (("pos", 1e-13), ("vel", 1e-12), ("F", 1e-13), ("stress", 1e-6)):
        ctx.below(maxDiff(a, b, fields=(field,)), tol,
                  "ecart Linear / RegularQuadLinear sur '%s'" % field)


@test("T04", "conservation_masse", INVARIANT, doc="""
La masse totale et le volume initial total sont des invariants du calcul.
Le volume initial est ce qui casse au premier decoupage adaptatif (B11).""")
def t04(ctx):
    c = ctx.case("T04_masse")
    c.write("input.txt", header(tmax=0.05, confPeriod=1000) + HOOKE + GRID + "\n" + COLUMN)
    c.run()
    c0, c1 = c.conf(0), c.lastConf()
    ctx.close(c1.totalMass(), c0.totalMass(), 1e-12, "masse totale")
    ctx.close(c1.totalVol0(), c0.totalVol0(), 1e-12, "volume initial total")
    noNaN(ctx, c, "conservation")


@test("T05", "colonne_elastique_stable", INVARIANT, doc="""
Une colonne elastique posee sur une rangee de noeuds bloques, amortie par PIC,
doit converger vers un etat statique : la contrainte moyenne sigma_yy tend vers
-rho g h / 2 et rien ne diverge. Test large, destine a attraper les NaN et les
divergences plutot qu'a valider une valeur.""")
def t05(ctx):
    c = ctx.case("T05_colonne")
    c.write("input.txt",
            header(tmax=0.2, confPeriod=4000, pic="enablePIC 0.2") + HOOKE + GRID + "\n" + COLUMN)
    c.run()
    c0, c1 = c.conf(0), c.lastConf()
    noNaN(ctx, c, "colonne")

    h = 0.06
    rho = 2000.0
    g = abs(c1.gravity[1])
    want = -0.5 * rho * g * h            # moyenne sur la hauteur du poids propre
    got = c1.meanStress()[3]
    ctx.close(got, want, 0.35, "sigma_yy moyen a l'equilibre")

    # La colonne ne doit ni s'envoler ni traverser le plancher.
    ctx.expect(min(p.y for p in c1.MP) > -0.01, "des MP sont passes sous la grille")
    ctx.expect(max(p.y for p in c1.MP) < 0.1, "la colonne a decolle")
    ctx.below(max(abs(p.vy) for p in c1.MP), 0.05, "vitesse residuelle apres amortissement")


@test("T06", "reprise_identique", INVARIANT, doc="""
Reprise de calcul : un run direct jusqu'a t = 0.03 doit donner le meme etat
qu'un run arrete a t = 0.01 puis repris depuis le conf-file. Modele
HookeElasticity, dont tout l'etat est bien sauvegarde. Ce test detectera toute
variable d'etat ajoutee et oubliee dans save() (voir C3).""")
def t06(ctx):
    direct = ctx.case("T06a_direct")
    direct.write("input.txt", header(tmax=0.03, confPeriod=1000) + HOOKE + GRID + "\n" + COLUMN)
    direct.run()

    restart = ctx.case("T06b_reprise")
    shutil.copy(os.path.join(direct.dir, "conf1.txt"),
                os.path.join(restart.dir, "input.txt"))
    restart.run()

    a = direct.lastConf()      # t = 0.03
    b = restart.lastConf()
    ctx.close(b.t, a.t, 1e-9, "temps final de la reprise")
    d = maxDiff(a, b)
    ctx.below(d, 1e-9, "ecart entre le calcul direct et la reprise")


@test("T07", "convergence_en_dt", INVARIANT, doc="""
Le schema doit converger : diviser dt par deux, a temps physique identique, ne
doit changer les DEPLACEMENTS que d'une fraction de leur amplitude. Comparer
les positions absolues ne prouverait rien, la colonne ne bougeant que de
quelques micrometres. Un ecart qui explose signale une grandeur qui s'accumule
d'un pas sur l'autre au lieu d'etre reinitialisee (famille B2).""")
def t07(ctx):
    a = ctx.case("T07a_dt")
    a.write("input.txt", timedHeader(1e-5, 2000) + HOOKE + GRID + "\n" + COLUMN)
    a.run()
    b = ctx.case("T07b_dt2")
    b.write("input.txt", timedHeader(5e-6, 4000) + HOOKE + GRID + "\n" + COLUMN)
    b.run()

    a0, a1 = a.conf(0), a.conf(1)
    b0, b1 = b.conf(0), b.conf(1)
    ctx.close(b1.t, a1.t, 1e-9, "les deux calculs comparent le meme temps physique")
    ctx.expect(a1.t > 0.0, "comparaison faite a t = 0")

    da, db = displacements(a0, a1), displacements(b0, b1)
    amp = max(max(abs(u), abs(v)) for u, v in da)
    ctx.expect(amp > 1e-9, "deplacement trop faible (%.3g m) pour conclure" % amp)
    err = max(max(abs(u1 - u2), abs(v1 - v2)) for (u1, v1), (u2, v2) in zip(da, db))
    ctx.below(err / amp, 0.05,
              "ecart relatif des deplacements entre dt et dt/2 "
              "(amplitude %.3g m)" % amp)


@test("T08", "contact_pas_de_traversee", INVARIANT, doc="""
Un bloc lache au-dessus d'un plancher ne doit jamais le traverser, et son
energie mecanique ne doit pas augmenter. Test de base des lois de contact.""")
def t08(ctx):
    c = ctx.case("T08_rebond")
    c.write("input.txt",
            header(tmax=0.3, confPeriod=3000) + HOOKE + GRID + "\n"
            + "set_MP_grid 0 HK 2000.0 0.06 0.10 0.10 0.14 0.005\n"
            + "Obstacle Line 1 0.36 0.05 0.04 0.05 freeze\n"
            + "BoundaryForceLaw frictionalViscoElastic 1\n"
            + "set kn 0 1 1e6\nset kt 0 1 1e6\nset mu 0 1 0.4\nset viscRate 0 1 0.2\n")
    c.run()
    noNaN(ctx, c, "rebond")

    c0 = c.conf(0)
    E0 = sum(p.mass * (0.5 * (p.vx ** 2 + p.vy ** 2) + 10.0 * p.y) for p in c0.MP)
    for n in range(1, 100):
        path = os.path.join(c.dir, "conf%d.txt" % n)
        if not os.path.exists(path):
            break
        cn = Conf(path)
        ctx.expect(min(p.y for p in cn.MP) > 0.05 - 0.01,
                   "des MP ont traverse le plancher (y_min = %.4g) dans conf%d"
                   % (min(p.y for p in cn.MP), n))
        E = sum(p.mass * (0.5 * (p.vx ** 2 + p.vy ** 2) + 10.0 * p.y) for p in cn.MP)
        ctx.expect(E <= E0 * 1.05 + 1e-9,
                   "l'energie mecanique a augmente : %.6g -> %.6g (conf%d)" % (E0, E, n))


# ===========================================================================
# 2. Defauts connus  (doivent echouer aujourd'hui, passer apres correction)
# ===========================================================================

@test("T09", "critere_de_pas_de_temps", INVARIANT, doc="""
convergenceConditions doit retenir le critere le PLUS contraignant. Corrige le
2026-08-05 (B5) : le code prenait std::max la ou son propre commentaire disait
'the smallest', et calculait sqrt(massMin / knMax) avec knMax reste a -DBL_MAX
quand il n'y a aucun obstacle -- donc une racine de nombre negatif.

Deux verifications complementaires :
  a) materiau raide, le critere CFL doit ramener dt a sa valeur analytique ;
  b) materiau souple et sans obstacle, aucun critere n'est viole, le dt demande
     doit etre respecte tel quel -- c'est le chemin qui produisait un NaN.""")
def t09(ctx):
    # a) E = 1 GPa, nu = 0.42, rho = 2700 : celerite 1521 m/s, dt_CFL/2 = 9.27e-7,
    #    tres en-dessous du dt demande de 1e-5.
    E, nu, rho, size = 1.0e9, 0.42, 2700.0, 0.005
    c = ctx.case("T09a_cfl")
    c.write("input.txt",
            timedHeader(1e-5, 200) + "model HookeElasticity HK %g %g\n" % (E, nu) + GRID + "\n"
            + "set_MP_grid 0 HK %g 0.06 0.06 0.10 0.10 %g\n" % (rho, size))
    c.run()
    ray = math.sqrt(size * size / math.pi)      # rayon du disque de meme aire
    want = 0.5 * ray / math.sqrt((E / (1.0 - 2.0 * nu)) / rho)
    ctx.close(c.conf(1).dt, want, 0.02, "dt ramene au critere CFL")

    # b) Le materiau souple par defaut donne dt_CFL/2 = 1.26e-5 : un dt de 1e-6
    #    passe largement, et l'absence d'obstacle ne doit pas produire de NaN.
    c2 = ctx.case("T09b_intact")
    c2.write("input.txt", timedHeader(1e-6, 200) + HOOKE + GRID + "\n" + BLOCK)
    c2.run()
    ctx.close(c2.conf(1).dt, 1e-6, 1e-9, "dt laisse intact quand aucun critere n'est viole")
    noNaN(ctx, c2, "sans obstacle")


@test("T10", "gravite_constante_sous_rampe", INVARIANT, doc="""
Une GravityRamp dont les deux extremites sont EGALES ne doit rien changer :
la gravite reste constante et la chute libre est inchangee. Corrige le
2026-08-05 (B6) : le terme constant manquait dans l'interpolation, la gravite
tombait a zero pendant toute la duree de la rampe et le bloc etait en
apesanteur. Regler les deux extremites a la meme valeur rend le test
insensible a la formule exacte d'interpolation.""")
def t10(ctx):
    c = ctx.case("T10_rampe")
    c.write("input.txt",
            header(tmax=0.025) + HOOKE + GRID + "\n" + BLOCK
            + "Scheduled GravityRamp 0 -10 0.005 0 -10 0.010\n")
    c.run()
    c1 = c.lastConf()
    g, t = -10.0, c1.t
    for p in c1.MP[:4]:
        ctx.close(p.vy, g * t, 1e-6, "v_y du MP %d sous une rampe plate" % p.nb)


@test("T11", "travail_du_poids", INVARIANT, doc="""
Le spy EnergyBalance accumule le travail du poids a partir de (pos - prev_pos).
Sur une chute libre, le cumul doit valoir m g Delta_y. Corrige le 2026-08-05
(B3) : prev_pos n'etait jamais mis a jour par ModifiedLagrangian, si bien que
chaque pas ajoutait le deplacement TOTAL depuis le debut -- surestimation d'un
facteur 1154 pour 2500 pas, soit environ n/2.

Le calcul est calibre pour que le dernier enregistrement du spy et le conf-file
tombent au meme pas : sans cela on compare le travail cumule a un instant et le
deplacement a un autre. Il reste un pas d'ecart, le spy etant execute apres la
sauvegarde, d'ou une tolerance de 1 %.""")
def t11(ctx):
    nsteps = 2000
    c = ctx.case("T11_travail")
    c.write("input.txt",
            timedHeader(1e-5, nsteps) + HOOKE + GRID + "\n" + BLOCK
            + "Spy EnergyBalance %d energy.txt\n" % (nsteps // 10))
    c.run()
    c0, c1 = c.conf(0), c.conf(1)
    rows = c.spy("energy.txt")
    ctx.expect(rows, "le fichier energy.txt est vide")
    ctx.expect(c1.t > 0.0, "comparaison faite a t = 0")

    m = c0.totalMass()
    dy = c1.centerOfMass()[1] - c0.centerOfMass()[1]
    want = m * c1.gravity[1] * dy          # > 0 : le poids travaille en descendant
    got = rows[-1][5]
    ctx.close(got, want, 0.01, "travail cumule du poids a t = %.4g" % c1.t)


@test("T12", "usl_gradient_de_vitesse", INVARIANT, doc="""
Une colonne de module 10 MPa chargee par son poids propre travaille a une
deformation de l'ordre de 1e-4 : det(F) doit rester a 2 % de 1, quel que soit
le schema d'integration.

Corrige le 2026-08-05 (B2) : avec UpdateStressLast le gradient de vitesse
n'etait jamais remis a zero, il cumulait depuis le premier pas et le gradient
de transformation divergeait -- 19 % d'ecart des t = 0.02 s, et au-dela de
0.03 s le calcul explosait, dt s'effondrait et le temps n'avancait plus. La
remise a zero est desormais faite par MPMbox::updateVelocityGradient, pour
qu'aucun schema ne puisse l'oublier.""")
def t12(ctx):
    c = ctx.case("T12_usl")
    c.write("input.txt",
            timedHeader(1e-5, 2000, scheme="UpdateStressLast") + HOOKE + GRID + "\n" + COLUMN)
    c.run(timeout=60)
    c1 = c.conf(1)
    noNaN(ctx, c, "UpdateStressLast")
    ctx.expect(c1.t > 0.0, "comparaison faite a t = 0")
    worst = max(abs(p.detF() - 1.0) for p in c1.MP)
    ctx.below(worst, 0.02,
              "ecart maximal de det(F) a 1 avec UpdateStressLast a t = %.4g" % c1.t)


@test("T13", "splitting_ne_gele_pas_F", INVARIANT, doc="""
Activer splitting sans preciser shearLimit ne doit pas modifier la mecanique.
Corrige le 2026-08-05 (B1) : shearLimit valait 0 par defaut, donc la condition
|F.xy| > shearLimit etait vraie des que le cisaillement n'etait pas exactement
nul. F etait remis a l'identite a chaque pas et le materiau ne se deformait
plus du tout. La valeur par defaut est passee a -1, qui desactive le
mecanisme.""")
def t13(ctx):
    ref = ctx.case("T13a_sans_split")
    ref.write("input.txt", header(tmax=0.02, confPeriod=2000) + HOOKE + GRID + "\n" + COLUMN)
    ref.run()

    c = ctx.case("T13b_avec_split")
    c.write("input.txt",
            header(tmax=0.02, confPeriod=2000).replace("splitting 0", "splitting 1")
            + HOOKE + GRID + "\n" + COLUMN)
    c.run()

    a, b = ref.lastConf(), c.lastConf()
    deformed = max(abs(p.detF() - 1.0) for p in a.MP)
    ctx.expect(deformed > 1e-9,
               "cas de reference sans deformation : le test ne prouve rien")
    frozen = max(abs(p.detF() - 1.0) for p in b.MP)
    ctx.expect(frozen > 1e-12,
               "avec splitting, tous les det(F) valent exactement 1 : "
               "F est remis a l'identite a chaque pas")


@test("T14", "decoupage_adaptatif", INVARIANT, doc="""
Un decoupage doit conserver la matiere, produire des numeros uniques, et couper
chaque point AU PLUS UNE FOIS par pas de temps.

Corrige le 2026-08-05 (B11) : MP2 heritait du numero du point d'origine, et la
borne de la boucle etant relue a chaque tour, les points tout juste crees
etaient re-examines dans la meme passe et redecoupes aussitot -- le resultat
dependait donc de l'ordre de rangement des points.

Attention a la grandeur conservee : c'est la SOMME DES vol, l'aire reellement
occupee. La somme des vol0 n'est pas conservee et ne doit pas l'etre : vol0 est
l'empreinte de reference, laissee intacte au decoupage, et c'est det(F) qui
porte la division. C'est ce qui fait que 'vol = det(F) * vol0' reste vrai dans
UpdateStressFirst/Last et que 'size = sqrt(vol0)' reste correct a la relecture
d'un conf-file.

Le critere est regle au minimum pour declencher un decoupage des le premier
pas, et shearLimit reste a sa valeur par defaut (mecanisme desactive).""")
def t14(ctx):
    c = ctx.case("T14_decoupage")
    c.write("input.txt",
            timedHeader(1e-5, 1).replace("splitting 0", "splitting 1")
            + "splitCriterionValue 1.0\nMaxSplitNumber 5\n"
            + HOOKE + GRID + "\n" + BLOCK)
    c.run()
    c0, c1 = c.conf(0), c.conf(1)
    noNaN(ctx, c, "decoupage")

    # Un seul pas, donc un seul appel a adaptativeRefinement : chaque point
    # d'origine donne exactement deux points.
    ctx.expect(len(c1.MP) == 2 * len(c0.MP),
               "%d points apres un pas au lieu de %d : des points crees pendant "
               "la passe ont ete redecoupes dans la meme passe"
               % (len(c1.MP), 2 * len(c0.MP)))

    for p in c1.MP:
        ctx.expect(p.splitCount == 2,
                   "le MP %d a un splitCount de %d apres un seul decoupage" % (p.nb, p.splitCount))

    ctx.close(c1.totalMass(), c0.totalMass(), 1e-12, "masse totale apres decoupage")
    ctx.close(sum(p.vol for p in c1.MP), sum(p.vol for p in c0.MP), 1e-12,
              "aire totale apres decoupage")

    nbs = [p.nb for p in c1.MP]
    ctx.expect(len(set(nbs)) == len(nbs),
               "%d numeros distincts pour %d points : les numeros sont dupliques"
               % (len(set(nbs)), len(nbs)))


@test("T15", "kelvinvoigt_independant_de_dt", INVARIANT, doc="""
Dans un modele de Kelvin-Voigt la contrainte visqueuse vaut eta * eps_point :
elle est instantanee, donc le resultat ne doit pas dependre du pas de temps.
Corrige le 2026-08-05 (B7) : elle etait ajoutee au cumul a chaque pas, ce qui
equivaut a une rigidite supplementaire eta/dt. L'ecart entre dt et dt/2 est
passe de 47 % a 0.06 %.

On ne teste pas la valeur de la contrainte visqueuse, qui demanderait une
solution analytique, mais son independance au pas de temps -- c'est exactement
la propriete que le defaut violait.""")
def t15(ctx):
    kv = "model KelvinVoigt KV 1.0e7 0.3 100.0\n"
    col = ("set_MP_grid 0 KV 2000.0 0.06 0.0 0.10 0.06 0.005\n"
           "set_BC_line 0 0 20 1 1\n")
    a = ctx.case("T15a_dt")
    a.write("input.txt", timedHeader(1e-5, 2000) + kv + GRID + "\n" + col)
    a.run()
    b = ctx.case("T15b_dt2")
    b.write("input.txt", timedHeader(5e-6, 4000) + kv + GRID + "\n" + col)
    b.run()

    ca, cb = a.conf(1), b.conf(1)
    ctx.close(cb.t, ca.t, 1e-9, "les deux calculs comparent le meme temps physique")
    sa, sb = ca.meanStress()[3], cb.meanStress()[3]
    ctx.expect(abs(sa) > 1.0, "contrainte trop faible pour conclure (%.3g Pa)" % sa)
    ctx.close(sb, sa, 0.10,
              "sigma_yy moyen a t = %.4g : sensibilite au pas de temps" % ca.t)


@test("T16", "frottement_pas_de_reptation", INVARIANT, doc="""
Un bloc pose sur un plan incline a 18 degres avec mu = 1.0 (largement au-dessus
de tan(18) = 0.33) doit s'immobiliser, et la frequence de reconstruction des
listes de voisins -- un reglage purement numerique -- ne doit rien y changer.

Corrige le 2026-08-05 (B9 et B10). La reconstruction perdait l'historique
tangentiel des contacts : le bloc reptait a -9.9 mm/s avec proxPeriod 10 contre
-0.21 mm/s avec proxPeriod 1000, soit un facteur 47 entre deux reglages qui
n'auraient pas du se voir. Les deux valeurs coincident desormais a la
6e decimale.

Le calcul va jusqu'a t = 1.5 s et la derive est mesuree sur le dernier tiers
seulement : le bloc est lache au-dessus de la pente et le transitoire de mise
en place, qui n'a rien a voir avec le frottement, dure environ 0.8 s.""")
def t16(ctx):
    def drift(name, proxPeriod):
        c = ctx.case(name)
        c.write("input.txt",
                # finalTime volontairement au-dela du dernier multiple de
                # confPeriod, sinon le cumul t += dt fait manquer le dernier
                # conf-file (defaut D11).
                header(tmax=1.55, confPeriod=30000, pic="enablePIC 0.3")
                .replace("proxPeriod 10", "proxPeriod %d" % proxPeriod)
                + HOOKE + GRID + "\n"
                + "set_MP_grid 0 HK 2000.0 0.15 0.13 0.19 0.16 0.005\n"
                + "Obstacle Line 1 0.35 0.15 0.05 0.05 freeze\n"
                + "BoundaryForceLaw frictionalViscoElastic 1\n"
                + "set kn 0 1 1e6\nset kt 0 1 1e6\nset mu 0 1 1.0\nset viscRate 0 1 0.5\n")
        c.run()
        noNaN(ctx, c, "reptation (proxPeriod %d)" % proxPeriod)

        confs = []
        for n in range(200):
            path = os.path.join(c.dir, "conf%d.txt" % n)
            if not os.path.exists(path):
                break
            confs.append(Conf(path))
        ctx.expect(len(confs) >= 4, "pas assez de conf-files (%d)" % len(confs))
        start, end = confs[-2], confs[-1]
        ctx.expect(end.t > 1.0, "mesure faite trop tot (t = %.3g)" % end.t)
        return (end.centerOfMass()[0] - start.centerOfMass()[0]) / (end.t - start.t)

    fast = drift("T16a_prox10", 10)
    slow = drift("T16b_prox1000", 1000)

    # Assertion principale : la physique ne doit pas dependre de proxPeriod.
    # C'est elle qui discrimine B9/B10 -- l'ecart valait 9.7 mm/s avant
    # correction, il vaut 0 aujourd'hui.
    ctx.below(fast - slow, 1e-4,
              "ecart de derive entre proxPeriod 10 (%+.4g m/s) et proxPeriod 1000 (%+.4g m/s)"
              % (fast, slow))

    # Assertion secondaire : le bloc est bien a l'arret. Le residu est du
    # tassement du contact penalise, pas du glissement (il decroit, et sa
    # composante verticale est plus grande que ce que la pente donnerait).
    for v, p in ((fast, 10), (slow, 1000)):
        ctx.below(v, 2e-3, "derive residuelle du bloc avec proxPeriod %d (m/s)" % p)


# ===========================================================================
# 3. Robustesse : refuser proprement une entree invalide
# ===========================================================================

def robust(ctx, name, content, what, expectMsg=None):
    """Le programme doit sortir avec un code non nul, sans mourir d'un signal.

    `expectMsg` : fragment attendu dans le message, pour s'assurer que l'arret
    a bien lieu pour la raison prevue et pas pour une autre.
    """
    c = ctx.case(name)
    c.write("input.txt", content)
    rc, out = c.run(expectFailure=True, timeout=60)
    if rc is None:
        raise Fail("%s : le programme ne s'arrete pas (timeout)" % what)
    if rc < 0:
        raise Fail("%s : le programme meurt sur %s au lieu de refuser l'entree\n%s"
                   % (what, describeRC(rc), tail(out)))
    if rc == 0:
        raise Fail("%s : l'entree invalide est acceptee silencieusement "
                   "(code de retour 0)" % what)
    if expectMsg and expectMsg not in out:
        raise Fail("%s : arret avec le code %d, mais le message ne contient pas "
                   "'%s' -- l'arret pourrait avoir une autre cause\n%s"
                   % (what, rc, expectMsg, tail(out)))


@test("T17", "schemas_dintegration_equivalents", INVARIANT, doc="""
Les trois schemas d'integration doivent decrire la meme mecanique. Sur une
colonne elastique chargee par son poids propre, la deformation reste de l'ordre
de 1e-4 : det(F) doit rester tres proche de 1 pour chacun d'eux.

Corrige le 2026-08-05 (B15) : UpdateStressLast lisait les vitesses nodales du
DEBUT de pas pour construire son increment de deformation, alors que tout
l'interet d'un schema 'update stress last' est d'utiliser le champ de fin de
pas. Il divergeait et finissait par ejecter un point hors de la grille.
Mesures a t = 0.4 s apres correction : ModifiedLagrangian 2.3e-4,
UpdateStressFirst 4.3e-5, UpdateStressLast 7.2e-5.""")
def t17(ctx):
    for scheme in ("ModifiedLagrangian", "UpdateStressFirst", "UpdateStressLast"):
        c = ctx.case("T17_" + scheme)
        c.write("input.txt",
                timedHeader(1e-5, 8000, scheme=scheme) + HOOKE + GRID + "\n" + COLUMN)
        rc, out = c.run(expectFailure=True, timeout=120)
        ctx.expect(rc == 0, "%s : arret premature (%s)\n%s" % (scheme, describeRC(rc), tail(out)))
        c1 = c.conf(1)
        worst = max(abs(p.detF() - 1.0) for p in c1.MP)
        ctx.below(worst, 0.01, "%s : ecart maximal de det(F) a 1 a t = %.4g" % (scheme, c1.t))


@test("T18", "parite_des_schemas", INVARIANT, doc="""
Les fonctionnalites ne doivent pas etre reservees a un seul schema
d'integration. Deux d'entre elles sont verifiees ici pour les trois schemas :

  a) la masse totale est conservee. Elle ne l'est que si la densite suit le
     volume : mass = vol * density, et c'est vol * density que le conf-file
     permet de recalculer.
  b) l'amortissement PIC agit. Compare l'energie cinetique residuelle d'une
     colonne elastique avec 'disablePIC' et avec 'enablePIC 0.9'.

Mise a niveau du 2026-08-05 : UpdateStressFirst et UpdateStressLast ne
connaissaient ni le melange FLIP/PIC ni la mise a jour de la densite. Mesures
sur UpdateStressFirst avant : energie cinetique strictement identique avec et
sans PIC (facteur 1, le mot-cle etait sans effet) et derive de masse de
6.3e-5. Apres : facteur 2.5e10 et derive nulle.""")
def t18(ctx):
    col = ("set_MP_grid 0 HK 2000.0 0.06 0.0 0.10 0.06 0.005\n"
           "set_BC_line 0 0 20 1 1\n")

    def run(scheme, pic):
        name = "T18_%s_%s" % (scheme, "PIC" if pic else "noPIC")
        c = ctx.case(name)
        c.write("input.txt",
                timedHeader(1e-5, 10000, scheme=scheme,
                            pic="enablePIC 0.9" if pic else "disablePIC")
                + HOOKE + GRID + "\n" + col)
        c.run()
        c0, c1 = c.conf(0), c.conf(1)
        ctx.expect(c1.t > 0.0, "%s : comparaison faite a t = 0" % name)
        kin = sum(0.5 * p.mass * (p.vx ** 2 + p.vy ** 2) for p in c1.MP)
        return c0.totalMass(), c1.totalMass(), kin

    for scheme in ("ModifiedLagrangian", "UpdateStressFirst", "UpdateStressLast"):
        m0, m1, kinFree = run(scheme, False)
        ctx.close(m1, m0, 1e-12, "%s : masse totale (somme des vol x density)" % scheme)

        _, _, kinPIC = run(scheme, True)
        ctx.expect(kinFree > 1e-12,
                   "%s : energie cinetique trop faible sans PIC (%.3g) pour conclure" % (scheme, kinFree))
        ctx.expect(kinPIC < 1e-3 * kinFree,
                   "%s : l'amortissement PIC est sans effet -- energie cinetique %.4g avec PIC "
                   "contre %.4g sans (facteur %.3g, attendu au moins 1000)"
                   % (scheme, kinPIC, kinFree, kinFree / kinPIC if kinPIC else float("inf")))


@test("T19", "see_ouvre_les_conf_files", INVARIANT, doc="""
Le visualiseur doit pouvoir ouvrir un conf-file sans mourir.

'see' partage tout le code de lecture avec mpmbox, et MPMbox::postProcess avec
la boucle de calcul, mais il n'a ni les memes initialisations ni le meme
enchainement. Un START_TIMER ajoute dans updateLiveNodeList l'a fait planter par
dereferencement d'un pointeur nul : INIT_TIMERS n'etait appele que dans
Runners/run.cpp. Rien dans la suite ne couvrait 'see', d'ou ce test.

Il n'est pas possible d'aller plus loin sans contexte graphique : une fois le
conf-file lu, 'see' entre dans la boucle GLUT et n'en sort plus. Le test lui
laisse quelques secondes et verifie qu'il est TOUJOURS VIVANT quand on le tue --
c'est-a-dire qu'il a passe la lecture et le post-traitement.""")
def t19(ctx):
    exe = os.path.join(os.path.dirname(ctx.exe), "see")
    if not os.path.isfile(exe):
        raise Fail("l'executable 'see' est introuvable a cote de mpmbox (%s)" % exe)

    c = ctx.case("T19_see")
    c.write("input.txt", timedHeader(1e-5, 200, sf="BSpline") + HOOKE + GRID + "\n"
            + "set_MP_grid 0 HK 2000.0 0.08 0.06 0.12 0.10 0.005\n")
    c.run()

    for conf in ("conf0.txt", "conf1.txt"):
        p = subprocess.Popen([exe, conf], cwd=c.dir,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        deadline = time.time() + 2.5
        while p.poll() is None and time.time() < deadline:
            time.sleep(0.2)
        rc = p.poll()
        if rc is None:
            # toujours vivant : il a franchi la lecture et le post-traitement
            p.kill()
            p.wait()
            continue
        out = p.stdout.read().decode("utf-8", "replace")
        raise Fail("see %s s'est arrete avec %s\n%s" % (conf, describeRC(rc), tail(out)))


@test("T28", "retrait_de_points_materiels", INVARIANT, doc="""
Le scheduler RemoveMaterialPoint compacte le tableau des points materiels.
Or les listes de voisins des obstacles reperent les points par leur INDICE
dans ce tableau : apres compactage, ces indices designent d'autres points, ou
sortent du tableau.

L'ordre dans MPMbox::run est checkProximity, puis les schedulers, puis le pas
de calcul : les points sont donc retires APRES la reconstruction des listes et
AVANT le pas qui les utilise. La detection par MP.size() != number_MP_before
n'intervient qu'au pas suivant, trop tard.

Le cas retire la majorite des points, pour que les indices survivants soient
loin au-dela de la nouvelle taille du tableau.""")
def t28(ctx):
    c = ctx.case("T28_retrait")
    c.write("input.txt",
            timedHeader(1e-5, 2000)
            + "model HookeElasticity HK 1.0e7 0.3\n"
            + "model HookeElasticity ATTENDU 1.0e7 0.3\n"
            + GRID + "\n"
            # le gros bloc part, le petit reste : ses indices sont les plus hauts
            + "set_MP_grid 0 HK 2000.0 0.06 0.10 0.34 0.16 0.005\n"
            # pose sur l'obstacle : c'est ce bloc, et lui seul, qui peuple la
            # liste de voisins (Line retient dstn < 2*size, soit 0.01 ici)
            + "set_MP_grid 0 ATTENDU 2000.0 0.06 0.03 0.10 0.05 0.005\n"
            + "Obstacle Line 1 0.36 0.03 0.04 0.03 freeze\n"
            + "BoundaryForceLaw frictionalViscoElastic 1\n"
            + "set kn 0 1 1e6\nset kt 0 1 1e6\nset mu 0 1 0.4\nset viscRate 0 1 0.2\n"
            + "Scheduled RemoveMaterialPoint HK 0.005\n")
    c.run()
    c0, c1 = c.conf(0), c.conf(1)
    noNaN(ctx, c, "retrait de points")
    ctx.expect(len(c1.MP) < len(c0.MP),
               "aucun point n'a ete retire : le test ne prouve rien")
    for p in c1.MP:
        ctx.expect(p.model == "ATTENDU",
                   "le MP %d porte le modele '%s' : le mauvais bloc a ete retire" % (p.nb, p.model))


@test("T29", "reactivation_des_liens_sans_dem", INVARIANT, doc="""
Le scheduler ReactivateCHCLBonds appelle PBC->ActivateBonds sur TOUS les points
materiels. PBC vaut nullptr pour un point simple echelle : un calcul sans
modele CHCL ou ce scheduler traine plante.""")
def t29(ctx):
    c = ctx.case("T29_reactivation")
    c.write("input.txt",
            timedHeader(1e-5, 500) + HOOKE + GRID + "\n" + BLOCK
            + "Scheduled ReactivateCHCLBonds 1.0e-5 0.002\n")
    c.run()
    noNaN(ctx, c, "reactivation sans DEM")
    ctx.expect(len(c.conf(1).MP) == 64, "les points ont disparu")


@test("T20", "modele_inconnu", INVARIANT, doc="""
Une faute de frappe sur le nom d'un modele doit produire un message et un arret
propre. Corrige le 2026-08-05 (A4) : l'iterateur de fin de la map etait
deference, dans cinq endroits differents.""")
def t20(ctx):
    robust(ctx, "T20_modele", header(tmax=0.001) + HOOKE + GRID + "\n"
           + "set_MP_grid 0 TYPO 2000.0 0.06 0.06 0.10 0.10 0.005\n",
           "modele inconnu", expectMsg="'TYPO' is not defined")


@test("T21", "loi_de_contact_inconnue", INVARIANT, doc="""
Un nom de BoundaryForceLaw inconnu doit etre refuse. Corrige le 2026-08-05
(A6) : il remplacait la loi par defaut par un pointeur nul, deference au
premier pas.""")
def t21(ctx):
    robust(ctx, "T21_bfl", header(tmax=0.001) + HOOKE + GRID + "\n" + BLOCK
           + "Obstacle Line 1 0.36 0.05 0.04 0.05 freeze\n"
           + "BoundaryForceLaw celleQuiNexistePas 1\n"
           + "set kn 0 1 1e6\nset kt 0 1 1e6\nset mu 0 1 0.4\nset viscRate 0 1 0.2\n",
           "loi de contact inconnue", expectMsg="is unknown")


@test("T22", "parametres_dinteraction_manquants", INVARIANT, doc="""
Des MP du groupe 2 face a un obstacle du groupe 1 alors que seul le couple
(0,1) est renseigne. Corrige le 2026-08-05 (A5) : DataTable::get ne verifie
aucune borne et le nombre de groupes ne grandit que par le mot-cle 'set', si
bien que le couple absent lisait hors des vecteurs.""")
def t22(ctx):
    robust(ctx, "T22_set", header(tmax=0.01) + HOOKE + GRID + "\n"
           + "set_MP_grid 2 HK 2000.0 0.06 0.08 0.10 0.12 0.005\n"
           + "Obstacle Line 1 0.36 0.05 0.04 0.05 freeze\n"
           + "BoundaryForceLaw frictionalViscoElastic 1\n"
           + "set kn 0 1 1e6\nset kt 0 1 1e6\nset mu 0 1 0.4\nset viscRate 0 1 0.2\n",
           "parametres d'interaction manquants",
           expectMsg="MP group 2, obstacle group 1")


@test("T23", "periode_nulle", INVARIANT, doc="""
confPeriod 0 est une division entiere par zero. Corrige le 2026-08-05 (A10).
Le symptome dependait du processeur : SIGFPE sur x86-64, mais sur AArch64 sdiv
par zero retourne 0 sans pieger, donc la condition etait toujours vraie et un
conf-file etait ecrit a chaque pas.""")
def t23(ctx):
    robust(ctx, "T23_periode",
           header(tmax=0.001).replace("confPeriod 2000", "confPeriod 0")
           + HOOKE + GRID + "\n" + BLOCK,
           "confPeriod nul", expectMsg="confPeriod = 0")


@test("T24", "conditions_limites_hors_grille", INVARIANT, doc="""
set_BC_line avec un numero de ligne superieur a Ny ecrivait en dehors du
vecteur de noeuds. Corrige le 2026-08-05 (A9) : les deux commandes de
conditions limites verifient leurs indices, et s'arretent au lieu de se
contenter d'un avertissement quand la grille n'existe pas encore.""")
def t24(ctx):
    robust(ctx, "T24_bc", header(tmax=0.001) + HOOKE + GRID + "\n" + BLOCK
           + "set_BC_line 99 0 5 1 1\n",
           "condition limite hors grille", expectMsg="outside the grid")


@test("T26", "parametre_set_inconnu", INVARIANT, doc="""
DataTable::set CREE le parametre quand le nom est inconnu : 'set Kn ...' etait
donc accepte sans un mot, pendant que le vrai kn restait a zero -- donc des
contacts sans raideur, et des points qui traversent les obstacles. Corrige le
2026-08-05 (C12).""")
def t26(ctx):
    robust(ctx, "T26_set_typo", header(tmax=0.001) + HOOKE + GRID + "\n" + BLOCK
           + "Obstacle Line 1 0.36 0.05 0.04 0.05 freeze\n"
           + "BoundaryForceLaw frictionalViscoElastic 1\n"
           + "set Kn 0 1 1e6\nset kt 0 1 1e6\nset mu 0 1 0.4\nset viscRate 0 1 0.2\n",
           "parametre d'interaction mal orthographie",
           expectMsg="unknown interaction parameter")


@test("T27", "noeuds_avant_la_grille", INVARIANT, doc="""
Une section 'Nodes' placee avant la creation de la grille indexait un vecteur
vide, et un numero de noeud hors grille n'etait pas verifie. Le cas se produit
avec un conf-file tronque ou edite a la main. Corrige le 2026-08-05 (C14).""")
def t27(ctx):
    robust(ctx, "T27_nodes", header(tmax=0.001) + HOOKE
           + "Nodes 2\n0 0 0 0 0 0 0 0 0 0\n1 0 0 0 0 0 0 0 0 0\n"
           + GRID + "\n" + BLOCK,
           "section Nodes avant la grille", expectMsg="before the grid")


@test("T25", "point_sorti_de_la_grille", INVARIANT, doc="""
Un point materiel lance hors de la grille doit provoquer un message clair et un
arret propre. Corrige le 2026-08-05 (A1) : ShapeFunction::locateElement borne
desormais l'indice d'element, qui n'etait verifie nulle part. Avant correction,
ce cas tuait le processus par SIGBUS.""")
def t25(ctx):
    robust(ctx, "T25_sortie", header(tmax=0.02, g=0.0) + HOOKE + GRID + "\n" + BLOCK
           + "prescribedVelocity 0 50.0 0.0\n",
           "point sorti de la grille",
           expectMsg="has left the grid")


# ===========================================================================
# Lanceur
# ===========================================================================

def findExe(given):
    if given:
        return os.path.abspath(given)
    env = os.environ.get("MPMBOX")
    if env:
        return os.path.abspath(env)
    for cand in ("BUILD/mpmbox", "build/mpmbox", "../BUILD/mpmbox", "bld/mpmbox"):
        p = os.path.join(ROOT, cand)
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    found = shutil.which("mpmbox")
    if found:
        return found
    return None


SRCDIRS = ("Core", "OneStep", "ConstitutiveModels", "ShapeFunctions", "Obstacles",
           "BoundaryForceLaw", "Commands", "Spies", "Schedulers", "Runners")


def staleSources(exe):
    """Sources plus recentes que l'executable.

    Un binaire perime donne des resultats qui ne veulent rien dire : le test
    d'un defaut tout juste corrige resterait au rouge, ou l'inverse.
    """
    try:
        tExe = os.path.getmtime(exe)
    except OSError:
        return []
    newer = []
    for d in SRCDIRS:
        full = os.path.join(ROOT, d)
        if not os.path.isdir(full):
            continue
        for f in os.listdir(full):
            if f.endswith((".cpp", ".hpp")):
                p = os.path.join(full, f)
                if os.path.getmtime(p) > tExe:
                    newer.append(p)
    newer.sort(key=os.path.getmtime, reverse=True)
    return newer


C = {"pass": "\033[32m", "fail": "\033[31m", "xfail": "\033[33m",
     "xpass": "\033[36m", "dim": "\033[2m", "off": "\033[0m", "bold": "\033[1m"}
if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
    C = {k: "" for k in C}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("select", nargs="*", help="identifiants a executer (T01, T10, ...)")
    ap.add_argument("-k", metavar="MOTIF", help="filtrer sur le nom")
    ap.add_argument("-l", "--list", action="store_true", help="lister les tests")
    ap.add_argument("--exe", help="chemin de l'executable mpmbox")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="avec -l : afficher la description ; sinon : afficher le "
                         "detail des XFAIL (utile pour verifier qu'un test echoue "
                         "bien pour la raison prevue)")
    args = ap.parse_args()

    chosen = TESTS
    if args.select:
        want = {s.upper() for s in args.select}
        chosen = [t for t in chosen if t["id"] in want]
    if args.k:
        chosen = [t for t in chosen if args.k.lower() in t["name"].lower()]

    if args.list:
        for t in chosen:
            bug = (" [%s]" % t["bug"]) if t["bug"] else ""
            print("%s  %-11s %-34s %s" % (t["id"], t["kind"], t["name"], bug))
            if args.verbose:
                print("    " + t["doc"].replace("\n", "\n    ") + "\n")
        return 0

    exe = findExe(args.exe)
    if not exe or not os.path.isfile(exe):
        print("Executable mpmbox introuvable. Utiliser --exe, ou la variable "
              "d'environnement MPMBOX.", file=sys.stderr)
        return 2

    print("%smpmbox%s : %s" % (C["bold"], C["off"], exe))
    stale = staleSources(exe)
    if stale:
        print("%sATTENTION%s : l'executable est plus ancien que %d fichier(s) source, "
              "dont %s.\n            Recompiler avant de conclure quoi que ce soit."
              % (C["fail"], C["off"], len(stale), os.path.relpath(stale[0], ROOT)))
    print("%stravail%s : %s\n" % (C["bold"], C["off"], WORK))
    os.makedirs(WORK, exist_ok=True)

    ctx = Ctx(exe)
    results = []
    t0 = time.time()

    for t in chosen:
        sys.stdout.write("  %-5s %-34s " % (t["id"], t["name"]))
        sys.stdout.flush()
        try:
            t["fn"](ctx)
            err = None
        except Fail as e:
            err = str(e)
        except Exception as e:                       # noqa: BLE001
            err = "%s : %s" % (type(e).__name__, e)

        expectFail = t["kind"] in (XFAIL, ROBUST)
        if err is None:
            status = "XPASS" if expectFail else "PASS"
        else:
            status = "XFAIL" if expectFail else "FAIL"
        results.append((t, status, err))

        col = {"PASS": C["pass"], "FAIL": C["fail"],
               "XFAIL": C["xfail"], "XPASS": C["xpass"]}[status]
        bug = (" %s(%s)%s" % (C["dim"], t["bug"], C["off"])) if t["bug"] else ""
        print("%s%-5s%s%s" % (col, status, C["off"], bug))
        if status == "FAIL":
            print("        " + err.replace("\n", "\n        "))
        elif status == "XFAIL" and args.verbose:
            print("        %s%s%s" % (C["dim"], err.replace("\n", "\n        "), C["off"]))
        elif status == "XPASS":
            print("        %sle defaut %s ne se manifeste plus : "
                  "basculer ce test en INVARIANT%s" % (C["dim"], t["bug"], C["off"]))

    wall = time.time() - t0
    counts = {}
    for _, s, _ in results:
        counts[s] = counts.get(s, 0) + 1

    print("\n%s%s%s" % (C["bold"], "-" * 62, C["off"]))
    print("  %d PASS   %d FAIL   %d XFAIL   %d XPASS      (%.1f s)"
          % (counts.get("PASS", 0), counts.get("FAIL", 0),
             counts.get("XFAIL", 0), counts.get("XPASS", 0), wall))

    if counts.get("FAIL"):
        print("\n%sREGRESSION%s : un invariant qui doit tenir avant comme apres "
              "les corrections est casse." % (C["fail"], C["off"]))
        for t, s, err in results:
            if s == "FAIL":
                print("  %s %s" % (t["id"], t["name"]))
    if counts.get("XPASS"):
        print("\n%sXPASS%s : ces defauts semblent corriges. Passer les tests "
              "correspondants en INVARIANT dans runtests.py." % (C["xpass"], C["off"]))
        for t, s, err in results:
            if s == "XPASS":
                print("  %s %s  (%s)" % (t["id"], t["name"], t["bug"]))

    if counts.get("XFAIL"):
        print("\n%s%d defaut(s) toujours presents%s : %s"
              % (C["dim"], counts["XFAIL"], C["off"],
                 ", ".join(sorted({t["bug"] for t, s, _ in results
                                   if s == "XFAIL" and t["bug"]}))))

    return 1 if counts.get("FAIL") else 0


if __name__ == "__main__":
    sys.exit(main())
