1. DESCRIPTION DU PROJET
   
1.1 L'équipe
   
Notre équipe est composée de trois étudiants aux compétences complémentaires en programmation, électronique et sciences appliquées.
Karl Kandakji – Responsable matériel et électronique Arduino
•	Assemblage du radar (servomoteur + capteur HC-SR04)
•	Programmation Arduino (balayage, mesure distance)
•	Calibration du système et tests de précision
•	Gestion de l'alimentation et sécurité électrique
Sarim Siddiqui – Responsable interface graphique et analyse
•	Interface graphique PyQt6 (radar + caméra)
•	Widget radar avec visualisation temps réel
•	Analyse mathématique (angles, coordonnées, seuils)
•	Documentation UML et présentation
Fady Kaldas – Responsable logiciel principal et vision artificielle
•	Programmation Python et OpenCV
•	Communication série Arduino ↔ Python
•	Logique de détection et activation caméra
•	Threading et optimisation performance
Cette répartition permet un travail en parallèle tout en assurant une intégration cohérente du système.
 
1.2 L'idée

Problématique

Comment concevoir un système automatisé capable de :
•	Détecter la présence d'un objet dans son environnement
•	Confirmer visuellement cette présence avec une caméra
•	Encadrer automatiquement l'objet détecté
•	Afficher le tout en temps réel dans une interface moderne
Le défi est d'intégrer plusieurs technologies (radar ultrason + vision artificielle + interface graphique) dans un système cohérent et pédagogique.
Objectifs du projet
•	Intégrer un capteur ultrason et une caméra dans une architecture unique
•	Mettre en place une logique par états (détection → alerte → activation caméra → encadrement)
•	Afficher les données radar et vidéo en temps réel côte à côte
•	Appliquer des notions de physique (ondes ultrasonores), mathématiques (angles, coordonnées) et informatique (threading, OpenCV)
•	Concevoir un système stable, fiable et adapté au niveau collégial
 
1.3 L'utilité

À quoi sert le système

Ce projet simule le fonctionnement d'un système de surveillance automatisé multi-capteurs similaire à ceux utilisés en robotique, en automatisation industrielle ou en systèmes de sécurité intelligents.
Le système scanne automatiquement une zone de 180° avec un radar ultrasonique. Dès qu'un objet est détecté à moins de 50 cm, la caméra s'active automatiquement et encadre l'objet avec un rectangle rouge grâce à OpenCV. Tout est affiché en temps réel dans une interface graphique montrant le radar et la vidéo.

Problème réel résolu

Dans les systèmes de surveillance réels, il est important de :

•	Réduire les fausses alarmes (faux positifs)
•	Confirmer une détection avant de déclencher une action
•	Économiser les ressources (la caméra ne tourne pas en continu)
•	Afficher clairement ce qui se passe pour l'utilisateur
Notre projet illustre cette chaîne décisionnelle de manière simplifiée, éducative et reproductible.
 
1.4 L'innovation

Nouveauté du projet

L'innovation repose sur la combinaison intelligente de deux technologies :
•	Un radar ultrason pour la détection large : Le servomoteur fait tourner le capteur HC-SR04 pour scanner 180° en continu
•	Une vision artificielle (OpenCV) pour la validation précise : La caméra s'active uniquement quand le radar détecte quelque chose, ce qui optimise les ressources

Valeur ajoutée

•	Architecture modulaire : Séparation claire entre Arduino (hardware), Python (traitement) et PyQt6 (interface)
•	Double validation : Radar + Caméra pour réduire les erreurs
•	Activation intelligente : Économie de ressources (caméra active seulement si nécessaire)
•	Interface temps réel : Visualisation simultanée du radar et de la caméra
•	Threading pour la performance : L'interface reste fluide même avec traitement vidéo
 
1.5 Cas d'utilisation
Acteurs
•	L'utilisateur (observateur) : Lance le système, configure les paramètres, observe les résultats
•	Le système automatisé : Détecte, confirme et affiche automatiquement
Scénario d'utilisation
1. Le système démarre en mode radar (balayage 0° → 180°)
2. Le capteur ultrason détecte un objet à 35 cm (seuil : 50 cm)
3. Arduino envoie "ALERT,90,35" via port série
4. Python reçoit l'alerte et active la caméra automatiquement
5. OpenCV détecte le mouvement et encadre l'objet avec un rectangle rouge
6. L'interface affiche le radar (point de détection) et la caméra (rectangle rouge) en temps réel
 
1.6 Public cible
•	Étudiants en génie électrique et électronique
•	Étudiants en informatique et programmation
•	Enseignants en sciences appliquées
•	Projets pédagogiques et démonstrations scientifiques
Le système est conçu pour être compréhensible, reproductible et adaptable à différents contextes éducatifs.
1.7 Liens avec les autres matières
Informatique
•	Programmation Python (structures de données, POO, threading)
•	Vision artificielle avec OpenCV (détection de mouvement, contours)
•	Communication série (protocole, parsing de données)
•	Interfaces graphiques avec PyQt6 (widgets, events, painting)
•	Architecture logicielle (UML, modularité, séparation des préoccupations)
Mathématiques
•	Trigonométrie (conversion angle/distance → coordonnées x,y)
•	Coordonnées polaires et cartésiennes
•	Analyse temporelle et seuils de détection
•	Calculs de distance (formule : distance = durée × 0.034 / 2)
Sciences (Physique)
•	Ondes ultrasonores (fréquence 40 kHz, vitesse du son)
•	Temps de vol et mesure de distance par réflexion
•	Conversion énergie électrique → mouvement (servomoteurs)
•	Notions de mécanique (rotation, angles, vitesse de balayage)
 
2. TECHNOLOGIES UTILISÉES
2.1 Outils et environnements
Langages
•	C/C++ : Programmation Arduino (contrôle servomoteur, lecture capteur HC-SR04)
•	Python 3.10 : Vision artificielle, communication série, interface graphique
IDE
•	Visual Studio Code : Environnement principal pour Arduino et Python
•	Extension Arduino pour VS Code : Upload et débogage du code Arduino
Bibliothèques principales
•	OpenCV : Traitement d'image et détection de mouvement
•	PYQt6 : Interface graphique moderne et widgets personnalisés
•	PySerial : Communication série Arduino ↔ Python
•	NumPy : Calculs mathématiques et traitement de données
Outils collaboratifs
•	Git : Contrôle de version du code
•	GitHub : Partage du code et collaboration en équipe
•	Google Docs : Documentation partagée
 
2.2 Justification des choix
   
•	Arduino : Simple, fiable et idéal pour le contrôle matériel. Grande communauté et documentation abondante.
•	Python : Adapté à la vision artificielle. Syntaxe claire et bibliothèques puissantes.
•	OpenCV : Bibliothèque standard en traitement d'image, gratuite et très documentée.
•	PyQt6 : Interface professionnelle et moderne. Permet des widgets personnalisés (radar circulaire).
•	VS Code : Un seul environnement pour Arduino et Python. Gratuit et léger.
•	Git/GitHub : Gestion efficace du travail en équipe. Historique complet des modifications.
Ces choix permettent une architecture claire séparant intelligence logicielle (Python) et contrôle physique (Arduino).

2.3 Défis et difficultés

•	Apprentissage de la vision artificielle : OpenCV nécessite de comprendre le traitement d'image (background subtraction, contours)
•	Synchronisation capteur/caméra : Coordonner Arduino et Python en temps réel sans décalage
•	Threading : Empêcher l'interface de se bloquer pendant la lecture série et capture vidéo
•	Précision du capteur : Le HC-SR04 a des limites (portée 2-400 cm, sensibilité aux angles)
•	Performance : Maintenir 30 FPS avec détection OpenCV active
•	Intégration matérielle/logicielle : Faire communiquer correctement toutes les parties du système
Ces défis contribuent à l'apprentissage technique et à la compréhension des systèmes embarqués.
 
4. PLAN DE TRAVAIL EN ÉQUIPE

3.1 Répartition des tâches

•	Karl Kandakji : Développement matériel et Arduino (~200 lignes C++)
•	Sarim Siddiqui : Interface graphique PYQt6 (~600-800 lignes Python)
•	Fady Kaldas : Communication série et vision OpenCV (~500-700 lignes Python)

3.2 Échéancier

Voir le diagramme de Gantt en annexe. Le projet s'étale sur 6 semaines (Semaine 4 à Semaine 9) :

Semaine 4 : Système caméra + détection OpenCV
Semaine 5 : Interface graphique complète (radar + caméra)
Semaine 6 : Tests et corrections
Semaine 7 : Améliorations avancées
Semaine 8 : Documentation complète
Semaine 9 : Présentation finale

3.3 Analyse du projet

Enjeux

•	Intégration multi-capteurs (radar + caméra)
•	Synchronisation matériel/logiciel en temps réel
•	Performance (30 FPS minimum)
•	Fiabilité de détection (réduire faux positifs)
Contraintes
•	Temps limité (6 semaines)
•	Budget restreint (~120 dollars)
•	Apprentissage de nouvelles technologies en parallèle
•	Limites techniques du matériel (précision capteur, qualité webcam)
 
3.4 Modélisation UML

Voir le diagramme UML complet en annexe. Le système est organisé en 6 modules principaux :

•	Arduino (Hardware) : RadarSystem, UltrasonicSensor, ServoMotor, SerialCommunication
•	Communication : ArduinoHandler (gestion port série)
•	Interface Principale : MainWindow (fenêtre 1200×600)
•	Widgets d'Affichage : RadarWidget, CameraWidget, ControlPanel
•	Détection OpenCV : MotionDetector, VideoRecorder
•	Threading : RadarThread, CameraThread
3.5 Vues de l'interface
L'interface graphique est divisée en trois zones principales affichées simultanément :
•	Zone Radar (gauche) : Cercle gradué 0-180°, ligne de balayage animée, points de détection en temps réel
•	Zone Caméra (droite) : Flux vidéo 30 FPS avec rectangles rouges autour des objets détectés
•	Panneau de Contrôle (bas) : Boutons (Connecter, Démarrer, Enregistrer), slider seuil de détection, barre de statut
 
5. CONCLUSION

Ce projet démontre l'intégration réussie d'un capteur ultrasonique, d'une caméra intelligente et d'une interface graphique moderne dans un système de surveillance automatisé. 
Nous avons conçu un système capable de scanner automatiquement une zone de 180°, de détecter les objets proches, d'activer une caméra intelligemment et d'encadrer visuellement les cibles en temps réel. Le tout est affiché dans une interface professionnelle montrant simultanément le radar et la vidéo.
Le projet illustre concrètement comment l'électronique (Arduino, capteurs), la programmation (Python, C++), les mathématiques (trigonométrie, coordonnées) et la physique (ondes ultrasonores) peuvent travailler ensemble pour créer un système intelligent, accessible et éducatif.

Compétences acquises :

•	Programmation embarquée avec Arduino
•	Vision artificielle avec OpenCV
•	Interfaces graphiques professionnelles avec PYQt6
•	Communication série et synchronisation temps réel
•	Architecture logicielle modulaire
•	Travail en équipe avec Git/GitHub
Extensions possibles :
•	Détection multi-objets avec YOLO
•	Reconnaissance faciale
•	Application web pour accès distant
•	Base de données pour historique des détections
•	Notifications par email/SMS
 
ANNEXES

Annexe A : Diagramme de Gantt
(Voir fichier Excel : Gantt_Projet_Radar_Surveillance.xlsx)
Annexe B : Diagramme UML
(Voir fichier image : UML_Radar_Simplifie.png)
Pour l'explication détaillée, voir : UML_Explication.txt
<img width="822" height="611" alt="image" src="https://github.com/user-attachments/assets/2bd9bfa7-9402-4ed7-9eb1-63794f3c682f" />
