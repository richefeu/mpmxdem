"""Lecture d'un conf-file MPMbox.

Le format d'un conf-file est celui d'un fichier d'entree : une suite de mots-cles.
Seuls ceux dont les tests ont besoin sont interpretes ; les autres sont conserves
tels quels dans `raw`.
"""

import math

# Colonnes d'une ligne de la section MPs, dans l'ordre ecrit par MPMbox::save().
# vec2r  -> 2 colonnes (x y)
# mat4r  -> 4 colonnes (xx xy yx yy)
MP_LAYOUT = [
    ("model", 1), ("nb", 1), ("group", 1),
    ("vol0", 1), ("vol", 1), ("rho", 1),
    ("pos", 2), ("vel", 2),
    ("strain", 4), ("plasticStrain", 4), ("stress", 4), ("stressCorrection", 4),
    ("splitCount", 1),
    ("F", 4),
    ("outOfPlaneStress", 1),
    ("contactf", 2),
]
MP_NCOL = sum(n for _, n in MP_LAYOUT)  # 34


class MP:
    """Un point materiel. Les tenseurs sont des tuples (xx, xy, yx, yy)."""

    __slots__ = [name for name, _ in MP_LAYOUT] + ["mass"]

    def __init__(self, tokens):
        if len(tokens) != MP_NCOL:
            raise ValueError("ligne MP : %d colonnes au lieu de %d" % (len(tokens), MP_NCOL))
        i = 0
        for name, n in MP_LAYOUT:
            chunk = tokens[i:i + n]
            i += n
            if name == "model":
                setattr(self, name, chunk[0])
            elif name in ("nb", "group", "splitCount"):
                setattr(self, name, int(chunk[0]))
            elif n == 1:
                setattr(self, name, float(chunk[0]))
            else:
                setattr(self, name, tuple(float(c) for c in chunk))
        self.mass = self.vol * self.rho

    @property
    def x(self):
        return self.pos[0]

    @property
    def y(self):
        return self.pos[1]

    @property
    def vx(self):
        return self.vel[0]

    @property
    def vy(self):
        return self.vel[1]

    def detF(self):
        return self.F[0] * self.F[3] - self.F[1] * self.F[2]


class Conf:
    """Etat complet lu dans un conf-file."""

    def __init__(self, path):
        self.path = str(path)
        self.t = 0.0
        self.dt = 0.0
        self.gravity = (0.0, 0.0)
        self.finalTime = 0.0
        self.Nx = self.Ny = 0
        self.lx = self.ly = 0.0
        self.MP = []
        self.nbObstacles = 0
        self.raw = []
        self._parse()

    def _parse(self):
        with open(self.path) as f:
            lines = f.readlines()

        i = 0
        while i < len(lines):
            line = lines[i]
            i += 1
            s = line.split()
            if not s or s[0][0] in "#/!":
                continue
            key = s[0]
            self.raw.append(line.rstrip("\n"))

            if key == "t":
                self.t = float(s[1])
            elif key == "dt":
                self.dt = float(s[1])
            elif key == "finalTime":
                self.finalTime = float(s[1])
            elif key == "gravity":
                self.gravity = (float(s[1]), float(s[2]))
            elif key == "set_node_grid" and s[1] == "Nx.Ny.lx.ly":
                self.Nx, self.Ny = int(s[2]), int(s[3])
                self.lx, self.ly = float(s[4]), float(s[5])
            elif key == "Obstacle":
                self.nbObstacles += 1
            elif key == "Nodes":
                i += int(s[1])          # les datasets nodaux ne sont pas relus ici
            elif key == "MPs":
                nb = int(s[1])
                for k in range(nb):
                    self.MP.append(MP(lines[i + k].split()))
                i += nb
            elif key == "ObstacleNeighbors":
                for _ in range(self.nbObstacles):
                    n = int(lines[i].split()[0])
                    i += 1 + n

    # --- quantites globales -------------------------------------------------

    def totalMass(self):
        return sum(p.mass for p in self.MP)

    def totalVol0(self):
        return sum(p.vol0 for p in self.MP)

    def centerOfMass(self):
        m = self.totalMass()
        if m == 0.0:
            return (0.0, 0.0)
        return (sum(p.mass * p.x for p in self.MP) / m,
                sum(p.mass * p.y for p in self.MP) / m)

    def meanStress(self):
        n = len(self.MP)
        if n == 0:
            return (0.0, 0.0, 0.0, 0.0)
        return tuple(sum(p.stress[k] for p in self.MP) / n for k in range(4))

    def maxAbsStress(self):
        return max((max(abs(v) for v in p.stress) for p in self.MP), default=0.0)

    def hasNonFinite(self):
        """Retourne la liste des (numero de MP, champ) non finis."""
        bad = []
        for p in self.MP:
            for name, n in MP_LAYOUT:
                if name == "model":
                    continue
                v = getattr(p, name)
                vals = v if isinstance(v, tuple) else (v,)
                for x in vals:
                    if not math.isfinite(x):
                        bad.append((p.nb, name))
                        break
        return bad


def maxDiff(confA, confB, fields=("pos", "vel", "stress", "F")):
    """Ecart maximal, MP par MP, entre deux configurations de meme taille."""
    if len(confA.MP) != len(confB.MP):
        raise ValueError("nombre de MP different : %d vs %d" % (len(confA.MP), len(confB.MP)))
    dmax = 0.0
    for a, b in zip(confA.MP, confB.MP):
        for name in fields:
            va, vb = getattr(a, name), getattr(b, name)
            if not isinstance(va, tuple):
                va, vb = (va,), (vb,)
            for xa, xb in zip(va, vb):
                dmax = max(dmax, abs(xa - xb))
    return dmax


def readColumns(path):
    """Lit un fichier de spy : retourne la liste des lignes, chacune une liste de floats."""
    rows = []
    with open(path) as f:
        for line in f:
            s = line.split()
            if not s or s[0].startswith("#"):
                continue
            try:
                rows.append([float(v) for v in s])
            except ValueError:
                continue
    return rows
