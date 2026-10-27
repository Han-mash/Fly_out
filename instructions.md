Le Makefile utilise le Python de `.venv` pour toutes ses cibles, donc il n'y a
rien à activer à la main. Un `make` ne peut pas activer un venv dans ton
terminal (il tourne dans un sous-processus). Si tu veux l'avoir dans ton
shell : `source .venv/bin/activate`.

La grille dit de lancer `mypy .`, qui regarde tous les `.py` du dossier.
Si `mypy` signale quelque chose, corrige avant la soutenance.

Fichiers du dépôt : le programme, `README.md`, `Makefile`,
`requirements.txt`, `.gitignore` et le dossier `maps/`. Le zip contient aussi
`check.py` et `benchmark.py`, deux outils de test à toi que le sujet ne
demande pas et que ni le Makefile ni le README ne mentionnent. La grille dit
« seuls les fichiers demandés » : retire-les du dépôt (`git rm`) ou garde-les
hors du dépôt. Décide-le avant, pas devant le correcteur. Tu peux toujours les
lancer à la main (`python3 check.py`, `python3 benchmark.py`) pour te rassurer.

## La grille, point par point

Pour compter les tours d'une carte sans fenêtre :

```
python3 fly_in.py --no-window -q maps/easy/01_linear_path.txt | wc -l
```

Pour toutes les cartes d'un coup :

```
for f in maps/*/*.txt; do echo "$f : $(python3 fly_in.py --no-window -q $f | wc -l) tours"; done
```

| Point de la grille | Quoi montrer | Résultat attendu |
|---|---|---|
| Bases / rendu | `git status`, `ls` | rien d'inattendu |
| README | ouvrir `README.md` | les 5 éléments demandés sont là (Description, Instructions, Algorithme, Représentation visuelle, Exemple) |
| POO | `models.py`, `scheduler.py`, `renderer.py` | classes séparées par rôle |
| Typage | `.venv/bin/python -m mypy .` | aucune erreur |
| Graphe maison | `grep -n "import" graph.py` | aucun `networkx`, `graphlib` |
| Parseur valide | `python3 fly_in.py --no-window maps/easy/02_simple_fork.txt` | 4 lignes de tours |
| Parseur erreurs | section 3 | message clair avec numéro de ligne |
| Capacité des zones | `maps/easy/03_basic_capacity.txt` | jamais plus de `max_drones` par zone (le simulateur le vérifie à chaque tour) |
| Coûts | `maps/medium/02_circular_loop.txt` | un token `D1-loop_b-exit_point` puis `D1-exit_point` : zone restricted = 2 tours |
| Capacité des liens | `maps/hard/02_capacity_hell.txt` | jamais plus de `max_link_capacity` par connexion |
| Visuel | `make run MAP=maps/medium/03_priority_puzzle.txt` | fenêtre pygame, couleurs des zones, drones animés |
| Scénarios simples | cartes `easy/` | tous sous les références |
| Fin de simulation | dernière ligne de la sortie | plus aucune ligne après l'arrivée du dernier drone |
| Chemins valides, conflits | cartes `medium/` et `hard/` | résolues, validées par le simulateur interne |
| Efficacité | la boucle ci-dessus | chaque carte répond en moins d'une seconde |
| Cartes faciles | boucle sur `maps/easy` | 4, 4, 4 tours (objectifs 6, 6, 8) |
| Cartes moyennes | boucle sur `maps/medium` | 8, 15, 7 tours (objectifs 15, 20, 12) |
| Cartes difficiles | boucle sur `maps/hard` | 13, 16, 26 tours (objectifs 45, 60, 35) |
| Cas limites | section 3 | gérés proprement |
| Live coding | section 5 | 10 minutes maximum |
| Bonus performance | boucle sur toutes les cartes | tous les objectifs tenus |
| Carte challenger | `maps/challenger/01_the_impossible_dream.txt` | 43 tours contre 45 de référence |

Le programme n'utilise aucun hasard : les nombres de tours sont les mêmes sur
toutes les machines.

Un détail que le correcteur peut relever : la grille parle de 3 drones pour
« fourche simple » et de 4 pour « casse-tête de priorité », mais les cartes
fournies en ont 4 et 5. On teste les cartes fournies, pas la grille.

## Cas d'erreur à montrer en direct

Crée ces fichiers dans un dossier à part (pas dans le dépôt) et lance
`python3 fly_in.py --no-window <fichier>`. Chaque cas doit finir par un
message sur stderr et un code de retour 1, sans trace d'exception.

| Fichier (contenu) | Message attendu |
|---|---|
| pas de `nb_drones` en premier | `nb_drones must come first.` |
| `end_hub: b 1 0 [zone=lava]` | `Unknown zone type 'lava' (expected one of: ...)` |
| aucun `start_hub` | `missing 'start_hub' zone.` |
| aucun `end_hub` | `missing 'end_hub' zone.` |
| `[max_drones=0]` | `max_drones must be a positive integer, got 0.` |
| `connection: a-b [max_link_capacity=-2]` | `max_link_capacity must be a positive integer, got -2.` |
| deux zones `a` | `zone name already used.` |
| `connection: a-b` puis `connection: b-a` | `duplicate connection` |
| `a-c` seulement, `b` isolé | `no route links the start zone to the end zone` |
| `end_hub` en `zone=blocked` | même message : le but est inatteignable |
| `nb_drones: 0` | `nb_drones must be a positive integer, got 0.` |
| fichier inexistant | `cannot read <nom>.` |
| `connection: a-a` | `Both ends are the same zone.` |

Capacités très élevées (`max_drones=1000000`, `max_link_capacity=999999`) :
tout passe, les drones avancent ensemble. Un seul drone : un plan simple, aucun
conflit possible.

Sans pygame (pour montrer la protection) :

```
python3 -m venv /tmp/nopg && /tmp/nopg/bin/python fly_in.py maps/easy/01_linear_path.txt
```

Les tours s'affichent, puis un avertissement sur stderr dit que la fenêtre n'a
pas pu s'ouvrir. Le programme finit avec le code 0.

## Questions probables et réponses

**Pourquoi un A\* dans l'espace-temps et pas un simple BFS ou Dijkstra ?**
Parce que la bonne réponse dépend du moment. Une zone libre au tour 3 peut être
pleine au tour 4. L'état de recherche est donc `(zone, tour)` et un drone peut
aussi attendre. Un Dijkstra sur le graphe seul ignore les capacités.

**À quoi sert la `DistanceMap` ?**
C'est un Dijkstra à l'envers depuis la zone d'arrivée, avec le coût d'entrée de
chaque zone. Elle donne la distance minimale s'il n'y avait personne. Elle sert
d'heuristique à l'A\* (elle ne surestime jamais, donc le premier but trouvé est
un des plus rapides) et elle détecte les cartes sans chemin.

**Comment gères-tu les zones `priority` ?**
Elles coûtent 1 tour comme une zone normale. Dans la recherche, un second score
départage les chemins de même durée : attendre ou entrer dans une zone normale
ajoute 1, une zone restricted ajoute 2, une zone priority ajoute 0. À durée
égale, on passe donc par les zones priority.

**Comment gères-tu les zones `restricted` ?**
Le drone met 2 tours : pendant le premier il est sur la connexion (on écrit
`D1-a-b`), il arrive au second (`D1-b`). Il n'est dans aucune zone pendant le
vol, mais il occupe la connexion pendant les deux tours. La capacité de la zone
d'arrivée est vérifiée au tour d'arrivée.

**Deux drones peuvent-ils échanger leurs places sur une connexion ?**
Non si la capacité est 1 : la table compte l'usage d'une connexion dans les deux
sens ensemble. Avec une capacité de 2, oui.

**Un drone peut-il entrer dans une zone de capacité 1 quand un autre en sort,
au même tour ?**
Oui. La capacité d'une zone est comptée à la fin du tour, et le drone qui part
n'y est plus. C'est aussi ce que vérifie le simulateur.

**Comment est planifiée toute la flotte ?**
Planification prioritaire : le drone 1 est planifié sur une table vide, son plan
est réservé, le drone 2 est planifié autour, etc. Un drone peut toujours
attendre sur la zone de départ (capacité illimitée), donc une solution existe
dans `rang x distance(départ, arrivée)` tours : c'est la borne de recherche.

**Ce n'est pas optimal, non ?**
Exact. Le glouton ne garantit pas le minimum. D'où la passe d'amélioration :
jusqu'à 3 tours de boucle, qui retirent le drone arrivé en dernier et un autre,
les replanifient (le dernier en premier) et gardent le résultat s'il est plus
court, ou aussi court et moins coûteux. Une recherche exacte multi-agents
(type CBS) serait plus lente et plus longue à écrire.

**Quelle complexité ?**
Distances : O((Z + E) log Z). Un A\* : au plus Z x T états, avec les voisins de
chacun, donc O(Z x T x deg x log(Z x T)). Pour D drones : D recherches, plus au
plus 3 x (D - 1) x 2 pour l'amélioration. Z zones, E connexions, T la borne de
tours.

**Pourquoi un simulateur séparé ?**
Pour ne pas se fier au planificateur. Le simulateur ne connaît que la carte et
le texte de sortie. S'il trouve une règle violée (capacité, connexion absente,
drone qui ne finit pas son vol...), il lève une erreur et rien n'est affiché
comme réussi.

**Comment est fait le graphe ?**
Trois structures : un dictionnaire nom -> zone, un dictionnaire clé -> connexion
où la clé est `frozenset({a, b})` (donc `a-b` et `b-a` sont la même connexion),
et une liste d'adjacence. `neighbors()` ne renvoie jamais les zones blocked.

**Pourquoi un `frozenset` comme clé ?**
Il est hachable et sans ordre. `{"a","b"} == {"b","a"}`, donc pas de doublon et
pas de cas à traiter dans les deux sens.

**Comment le parseur rejette-t-il les erreurs ?**
Il lève `ParseError` avec le numéro et le texte de la ligne. Il ne fait jamais
`print` ni `exit` : c'est `fly_in.py` qui affiche et décide du code de retour.
Les erreurs de fichier (introuvable, droits, encodage) sont `MapFileError`.

**Où est la POO ?**
Chaque rôle est une classe : modèles de données passifs (`Zone`, `Connection`,
`Drone`), `Graph`, `MapParser`, `DistanceMap`, `SpaceTimeAStar`, `Frontier`,
`ReservationTable`, `Leg` et `DronePlan`, `Scheduler`, `Simulator`,
`PygameRenderer`. Les exceptions héritent toutes de `FlyInError`, ce qui permet
un seul `except` dans le point d'entrée. `ZoneType` est une `Enum` qui porte
elle-même son coût d'entrée.

**Que se passe-t-il si pygame n'est pas installé ?**
`renderer.py` n'est importé qu'au moment d'ouvrir la fenêtre. Si l'import
échoue parce que `pygame` manque, on affiche un avertissement et on continue :
la sortie texte est déjà imprimée. Une autre erreur d'import n'est pas cachée.

**Et s'il n'y a pas d'écran ?**
`pygame.error` est transformée en `DisplayError`, affichée comme avertissement.
Le programme finit normalement.

**Pourquoi `os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")` ?**
Pygame écrit une ligne de bienvenue à l'import. Cette variable la supprime sans
toucher à `stdout`, ce qui garde la sortie propre.

**Pourquoi les statistiques sont sur stderr ?**
Pour que stdout ne contienne que les lignes de tours, que le sujet impose.

**Pourquoi `make install` crée un venv ?**
Pour ne pas installer pygame dans le Python du système. Toutes les cibles du
Makefile utilisent `.venv/bin/python`, donc elles marchent sans activation, et
`make run` lance même l'installation si le venv n'existe pas encore.

**Quelle est la plus grosse limite ?**
Le glouton avec priorité fixe par numéro de drone, et le temps de recherche qui
grossit avec Z x T sur de très grands graphes. Sur les cartes fournies, même la
carte challenger de 25 drones et 54 zones se résout en environ un dixième de
seconde.

**Qu'est-ce que tu ferais avec plus de temps ?**
Essayer plusieurs ordres de planification des drones et garder le meilleur,
ou une recherche en fenêtre (Windowed Hierarchical Cooperative A\*).

## `--capacity-info`

Consigne de la grille : `./fly_in.py --capacity-info map.txt` doit produire la
simulation normale plus, pour chaque tour, des lignes du type
`Zone X: Y/Z drones` et `Connection A-B: Y/Z capacity used`. 10 minutes
maximum, démonstration comprise.

L'idée : le planificateur utilise déjà une `ReservationTable` qui retient, pour
chaque tour, combien de drones sont dans chaque zone (`zone_load`) et sur
chaque connexion (`link_load`). On la reconstruit à partir des plans et on la
relit à chaque tour. Aucune méthode n'est à écrire, ni aucun fichier à créer :
6 lignes de logique, plus le flag et l'import. Ce n'est pas de la triche, tu
relis des compteurs qui existent déjà et qui servent à planifier. Dis-le
tel quel au correcteur : « la table de réservation stocke déjà la charge par
tour, je la relis ».

Trois modifications dans `fly_in.py`.

```python
# 1. en haut, avec les autres imports
from reservation import ReservationTable

# 2. dans App._parse_args, avant le return
parser.add_argument("--capacity-info", action="store_true")

# 3. dans App.run, à la place de la boucle qui imprime schedule.lines()
table = ReservationTable(graph)
for plan in schedule.plans:
    table.reserve_plan(plan)
for turn, line in enumerate(schedule.lines(), start=1):
    print(line, file=self._out)
    if args.capacity_info:
        for z in graph.zones:
            print(f"  Zone {z.name}: {table.zone_load(z, turn)}"
                  f"/{z.capacity or 'unlimited'} drones")
        for c in graph.connections:
            print(f"  Connection {c.zone_a.name}-{c.zone_b.name}: "
                  f"{table.link_load(c, turn)}/{c.max_link_capacity}"
                  " capacity used")
```

Test à montrer :

```
python3 fly_in.py --no-window -q --capacity-info maps/easy/02_simple_fork.txt
```

Début de sortie attendu :

```
D1-junction D2-junction
  Zone start: 2/unlimited drones
  Zone junction: 2/2 drones
  Zone path_a: 0/1 drones
  Zone path_b: 0/1 drones
  Zone goal: 0/unlimited drones
  Connection start-junction: 2/2 capacity used
  ...
```

Tout est affiché à chaque tour, y compris les `0/1`, ce qui montre aussi les
capacités. Sans le flag, la sortie est identique à avant.

**Un point que le correcteur peut relever.** Pour la zone d'arrivée, la table
ne compte que les drones qui arrivent pendant ce tour, pas ceux déjà livrés les
tours d'avant. Toutes les autres zones sont exactes. Pour corriger, ajoute cette
ligne avant les deux `for` internes, et remplace `table.zone_load(z, turn)` par
`pos.count(z.name)` :

```python
pos = list(result.history[turn - 1].positions.values())
```

Après la démonstration, `git stash` ou `git checkout fly_in.py` pour revenir à
la version propre si tu ne veux pas garder la modification.
