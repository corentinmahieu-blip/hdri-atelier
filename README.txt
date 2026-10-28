UTILISATION
Tenir l’iPhone en paysage. Activer caméra et capteurs. Fixer le départ une fois.
Laisser « Scan continu » coché. Balayer lentement les couronnes autour de l’objectif,
sans s’arrêter à chaque point. Les zones manquantes proches du centre sont capturées
quel que soit le sens du passage. Monter ensuite vers le plafond et descendre vers le sol.
Les repères restent nécessaires : ce n’est pas une reconstruction 3D libre.
Si le traitement prend du retard, ralentir et repasser sur les zones manquantes.
Pause permet d’interrompre le parcours. Décocher Scan continu permet la capture manuelle.
Les originaux sont conservés, l’aperçu se construit progressivement et l’assemblage final
se lance depuis l’export. Exporter les originaux avant de fermer ou d’effacer les données.

CHANGEMENTS
Sélection jusqu’à trois images au rythme des images vidéo décodées lorsque disponible.
Une sortie de zone conserve la meilleure image déjà admissible au lieu de mettre en pause.
Aucun flash ni vibration pour les prises automatiques. Aperçu calculé à partir de vues
réduites à 960 pixels de large ; originaux et assemblage final restent à leur résolution antérieure.
Transfert du tampon de pixels au worker sans copie explicite.
Seuil de vitesse prudent conservé : tourner vite peut produire des zones manquantes.

LIMITES
Prototype testé sous Node, pas sur Safari/iPhone réel. Ni cadence ni mémoire garanties.
Pas de vidéo enregistrée : sélection de photos dans le flux vidéo, sans envoi serveur.
Panorama SDR, sans LiDAR ni ARKit ni fusion HDR. Les raccords peuvent rester visibles.
La fluidité de capture ne garantit pas un meilleur assemblage.

DÉVELOPPEMENT
python3 build.py
node --test tests/*.test.*
Le banc de test utilise @napi-rs/canvas. Les scripts de test de session réelle exigent
les photos privées de test, qui ne sont pas incluses.
