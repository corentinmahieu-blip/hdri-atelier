# Validation V3.4

33 tests Node passent. Ajouts : sélection lors de sortie de zone, interruption,
cibles manquantes hors ordre, absence de doublon, callback vidéo et scénario
app de balayage en sens inverse. Le banc de test DOM/worker est simulé ;
les performances Safari, le cadrage réel et la mémoire iPhone ne sont pas validés.
Aucun nouveau gain de qualité du panorama final n’est revendiqué.

## Historique V3.3 (avant modification de la capture et de l’aperçu)

# HDRI Atelier V3.3 — validation

Tests réalisés le 28 octobre 2026 dans Node.js, sur les sources générées. Les scénarios utilisent un DOM simulé et un canevas 2D réel ; ce n'est pas Safari ni un iPhone.

- Tests géométriques : repères, orientation, couverture théorique, projection, passage ±180°, interruption et déclenchement.
- Tests ZIP : export ; import d'une session au format non compressé ; rejet d'une photo altérée ; aucune modification de la session active lors de l'import.
- Tests de sauvegarde : écriture et réouverture avec un double transactionnel d'IndexedDB, rejet des échecs et conservation de la photo précédente. La persistance réelle de Safari n'est pas validée ici.
- Scénarios applicatifs : capture, reprise, export, assemblage simulé, comparaison avant/après et annulation, sans suppression des photos originales.
- Ajustement global synthétique : récupération d'une orientation connue à partir de rayons communs ; vue déconnectée conservée avec son orientation initiale.
- Correction locale : vérification de la direction de l'échantillonnage et de la conservation des pixels originaux.
- Protocole du worker : lancement du code worker généré avec le véritable OpenCV.js et deux photos réelles, via un adaptateur Node (ce n'est pas un worker navigateur). Détection d'un raccord et production d'un atlas avec pixels non transparents.
- Session réelle de 33 photos : traitement du moteur JavaScript sur ordinateur. 49 paires et 3 061 correspondances retenues ; 26 vues reliées dans le groupe principal, 7 vues isolées ou dans d'autres groupes ; 21 corrections locales retenues selon les correspondances réservées. La fonction de coût globale tronquée diminue sur ces correspondances, ce qui ne prouve pas une correction parfaite. À l'examen du panorama, des ruptures d'éclairage et des mauvais raccords subsistent. C'est un essai, pas une validation de qualité finale.
- Le résidu des corrections locales est évalué sur un sous-ensemble de correspondances réservé, mais des points proches ou répétés peuvent se retrouver entre l'apprentissage et ce sous-ensemble. Il ne constitue donc pas une évaluation indépendante d'une nouvelle scène.

À tester : chargement OpenCV et worker sur Safari iPhone, accès au ZIP dans Fichiers, consommation mémoire, durée, interruption/annulation effective, sauvegarde/partage du PNG, mise en page en paysage et qualité sur de nouvelles scènes. Un navigateur Apple et un iPhone n'étaient pas disponibles dans cet environnement.

## Relancer

```
python3 build.py
node --test tests/*.test.mjs tests/*.test.cjs
```

Les tests de scénario et d'images synthétiques nécessitent @napi-rs/canvas, uniquement pour le développement. Le test réel tests/real-session.cjs requiert les photos originales, qui ne sont pas incluses. tests/worker-protocol.cjs nécessite également deux de ces photos et un adaptateur Node.
