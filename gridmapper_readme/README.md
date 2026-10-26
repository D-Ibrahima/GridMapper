# GridMapper

Simulation en C++/OpenCV d'un robot mobile qui **cartographie un environnement inconnu** avec un LiDAR 2D simulé. Le robot construit une **grille d'occupation** en temps réel et peut se piloter de trois façons : manuellement, en **exploration autonome** (détection de frontières + planification de chemin) ou en **suivi de mur**. Le changement de mode peut se faire au clavier ou en montrant un **marqueur ArUco** à la caméra.

![Démonstration](docs/demo.gif)

*Démonstration accélérée : à gauche, la carte réelle avec le robot (rouge) ; à droite, la grille d'occupation qu'il construit (gris = inconnu, blanc = libre, noir = mur).*

## Aperçu : exploration autonome

Le robot part en bas à droite avec une grille entièrement inconnue, puis explore la carte en allant de frontière en frontière. À chaque étape, la fenêtre de gauche montre la carte réelle et celle de droite la grille d'occupation reconstruite.

| 1. Départ | 2. Exploration en cours |
|:---:|:---:|
| ![Départ](docs/01-depart.png) | ![Exploration en cours](docs/02-exploration.png) |
| Le LiDAR a déjà révélé la moitié basse ; le haut de la carte reste inconnu (gris). | Le robot longe un mur et découvre la zone en haut à droite. |

| 3. Carte presque complète | 4. Carte finale |
|:---:|:---:|
| ![Carte presque complète](docs/03-presque-termine.png) | ![Carte finale](docs/04-carte-finale.png) |
| Il ne reste qu'une petite zone inconnue en haut à gauche. | Plus aucune frontière accessible : la carte est entièrement explorée et les murs sont continus. |

## Fonctionnalités

- **LiDAR simulé** : 360 rayons (1 par degré), portée de 300 px, lancer de rayons sur l'image de la carte.
- **Grille d'occupation** : mise à jour à chaque pas à partir du scan, avec remplissage de secteurs entre rayons voisins (pas de trous) et murs continus.
- **Exploration autonome** : recherche par BFS de la frontière « libre / inconnu » la plus proche par le chemin, avec marge de sécurité autour des murs, suivi du chemin et évitement d'obstacles à trois niveaux (imminent, proche, lointain), plus un anti-blocage.
- **Suivi de mur (à gauche)** : régulation de la distance au mur **et** de l'angle par rapport au mur (estimé avec deux rayons LiDAR), gestion des coins et de la perte du mur.
- **Mode manuel** : déplacement au clavier, avec collision contre les murs.
- **Changement de mode par ArUco** (dictionnaire 6x6_250) : ID `0` = manuel, ID `1` = exploration, ID `42` = suivi de mur.
- Fonctionne **sans caméra** (modes pilotés au clavier).

## Commandes

Dans la fenêtre « Map du Robot » :

| Touche | Action |
|---|---|
| `E` | Mode exploration |
| `F` | Mode suivi de mur |
| `M` | Mode manuel |
| `W` `A` `S` `D` | Déplacer le robot (mode manuel) |
| `R` | Remettre le robot au point de départ (mode manuel) |
| `Q` / `Échap` | Quitter |

Pour les marqueurs ArUco, il suffit de les présenter à la caméra : le mode change à l'**apparition** d'un marqueur (une touche du clavier reste donc prioritaire tant que le marqueur ne change pas).

## Installation

### Dépendances

- Compilateur C++17 (g++ ou clang++)
- CMake ≥ 3.10
- OpenCV avec le module ArUco :
  - OpenCV ≥ 4.7 (module `objdetect`), ou
  - OpenCV 4.x avec `opencv_contrib` (ancienne API, gérée automatiquement)

Sous Ubuntu/Debian :

```bash
sudo apt install build-essential cmake libopencv-dev
```

### Compilation et lancement

```bash
git clone https://github.com/D-Ibrahima/GridMapper.git
cd GridMapper
mkdir build && cd build
cmake ..
make
./program
```

Le programme doit être lancé depuis `build/` (ou depuis la racine du projet) : il cherche la carte dans `../res/img.png`, puis dans `res/img.png`.

### Dépannage

- **Messages `GStreamer` / `V4L2` / `Camera index out of range`** : aucune caméra n'est accessible à l'index 0. Le programme continue sans caméra ; si ta caméra est sur un autre index, change `Camera camera;` en `Camera camera(1);` dans `src/main.cpp`.
- **`symbol lookup error ... /snap/core20/.../libpthread.so.0`** : tu lances le programme depuis le terminal intégré d'un VS Code installé en snap, qui injecte des variables d'environnement vers le snap. Lance-le depuis un terminal système, ou avec :
  ```bash
  env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE \
      -u LOCPATH -u XDG_DATA_DIRS -u XDG_DATA_HOME ./program
  ```

## Fonctionnement

### Conventions

Repère image (axe y vers le bas). Les angles croissent dans le **sens horaire** à l'écran. Pour le LiDAR, le rayon `0` pointe devant le robot, `90` à droite, `180` derrière et `270` à gauche. Les vitesses sont en px/s (avance) et en degrés/s (rotation, positive = vers la droite).

### Boucle principale

À chaque pas (Δt = 0,033 s) : lecture caméra et détection ArUco → lecture du clavier → calcul de la commande selon le mode → déplacement → collision avec les murs → scan LiDAR à la **nouvelle** position → mise à jour de la grille → affichage.

### Exploration

1. Le plan est recalculé à chaque pas : BFS sur une version sous-échantillonnée de la grille (cellules de 5 px), en ne traversant que les cellules libres éloignées des murs.
2. La cible est la cellule libre voisine d'une cellule inconnue la plus proche par le chemin ; le robot vise un point de passage quelques cellules plus loin sur ce chemin.
3. Des règles de sécurité prennent le relais près des obstacles, et un anti-blocage fait reculer le robot s'il n'avance plus.

### Suivi de mur

Deux rayons latéraux (270° et 300°) donnent la distance perpendiculaire au mur et son angle. La commande de rotation combine l'erreur de distance (cible : 60 px) et l'erreur d'angle. Un obstacle devant fait pivoter le robot à droite ; un mur perdu le fait courber à gauche.

## Structure du projet

```
GridMapper/
├── CMakeLists.txt
├── res/img.png            # carte de l'environnement (noir = mur)
├── include/               # en-têtes
├── src/
│   ├── main.cpp           # boucle principale, affichage
│   ├── robot.cpp          # état et cinématique du robot
│   ├── lidar.cpp          # LiDAR simulé (lancer de rayons)
│   ├── occupancy_grid.cpp # grille d'occupation
│   ├── exploration.cpp    # exploration par frontières + BFS
│   ├── wallfollow.cpp     # suivi de mur
│   ├── behavior.cpp       # modes (manuel / exploration / suivi de mur)
│   ├── aruco_manager.cpp  # détection des marqueurs ArUco
│   ├── camera.cpp         # accès caméra
│   └── keyboard.cpp       # commandes manuelles
└── docs/                  # GIF de démonstration et captures d'écran
```

## Résultats en simulation

Sur la carte fournie (700×700 px), en simulation sans interface :

- **Exploration** : carte entièrement explorée en environ 12 000 pas (~6,6 min simulées), sans passer à moins de 5 px d'un mur.
- **Suivi de mur** : aucun contact avec un mur, trajectoire stable (pas de boucle sur place).

## Limites et pistes d'amélioration

- La position du robot est **connue** (pas d'odométrie bruitée ni de SLAM) : le projet fait de la cartographie, pas de la localisation.
- Le LiDAR est parfait (sans bruit) ; ajouter du bruit de mesure rendrait le test plus réaliste.
- Le suivi de mur ne suit que le mur de gauche ; dans une carte à îlots, il tourne autour d'un îlot sans tout parcourir.
- Les gains et vitesses sont réglés pour cette carte et ce pas de temps.
- Idées : bruit capteur, localisation par filtre à particules, choix de la carte en argument, mode « retour au départ ».

## Auteur

Ibrahima Sory Diakite, élève ingénieur en robotique à Polytech Dijon.
