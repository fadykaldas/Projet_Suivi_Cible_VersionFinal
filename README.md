# Projet_Suivi_Cible_VersionFinal

Système automatisé de détection et de suivi Présentation: Ce projet consiste en la conception d’un système automatisé basé sur Arduino capable de détecter une cible, confirmer sa stabilité, puis déclencher un mécanisme motorisé. Il intègre un capteur ultrason, une caméra et un servomoteur afin de démontrer l’interaction entre l’électronique, la programmation et la mécanique dans un prototype fonctionnel.

Problématique: Comment concevoir un système fiable capable de détecter une cible et d’agir uniquement lorsque sa présence est confirmée, tout en limitant les fausses détections ? La solution proposée repose sur une vérification de stabilité de 2 secondes avant l’activation du mécanisme.

Fonctionnement Le capteur ultrason mesure en continu la distance. Lorsqu’un objet entre dans la zone de détection, la caméra s’active. Un délai de 2 secondes vérifie la stabilité de l’objet. Si l’objet demeure stable, le servomoteur déclenche le mécanisme. En cas de mouvement, le système se réinitialise.

Technologies utilisées:

Matériel : Arduino UNO Capteur ultrason HC-SR04 Servomoteur Caméra

Logiciel : Arduino IDE (C++) Processing (interface de visualisation)

Objectif pédagogique

Ce projet illustre la transformation d’une mesure physique en donnée numérique, son traitement algorithmique, puis l’exécution d’une action mécanique contrôlée.

Il constitue un exemple concret d’intégration multidisciplinaire en sciences et ingénierie.
