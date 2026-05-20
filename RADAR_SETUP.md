# Intégration Capteur Radar HC-SR04 - Guide d'Installation

## 📌 Branchements Hardware

Connectez le capteur **HC-SR04** (ou compatible) comme suit sur la breadboard :

| Capteur HC-SR04 | Breadboard     | Arduino |
|---|---|---|
| **VCC** | Rail rouge (+5V) | - |
| **GND** | Rail bleu/noir (GND) | - |
| **TRIG** | - | Pin D7 |
| **ECHO** | - | Pin D6 |

⚠️ **IMPORTANT** : 
- Utilisez une **alimentation externe 5V** pour la breadboard, pas le 5V de l'Arduino
- Assurez-vous que les GND sont tous connectés ensemble (Arduino GND + GND breadboard + GND alimentation)

---

## 🔧 Installation Logicielle

### 1. Uploader le sketch Arduino
```bash
# Fichier modifié: arduino/face_tracker_servos.ino
# - Support du capteur HC-SR04 (pins D7 TRIG, D6 ECHO)
# - Moyenne mobile pour lisser les mesures
# - Envoi des distances toutes les 200ms au format: "dist:XX.XX"
```

### 2. Configurer l'application Python
L'application gère automatiquement les données du radar. Les modifications incluent :

- ✅ Parser mis à jour pour lire `dist:XX.XX` depuis l'Arduino
- ✅ Activation de la **caméra UNIQUEMENT dans l'onglet Live**
- ✅ Logique d'automatisation existante réutilisée

---

## ⚙️ Configuration Recommandée

Allez dans l'onglet **Settings** et réglez :

### Automatisation du Radar
| Paramètre | Valeur Défaut | Recommandation |
|---|---|---|
| ON threshold | 60 cm | **50-60 cm** (distance d'activation) |
| OFF threshold | 70 cm | **70-80 cm** (hysteresis pour éviter oscillations) |
| Confirm time | 2.0 s | **1.5-2.0 s** (délai avant activation) |
| Close after loss | 10.0 s | **8-10 s** (temps avant fermeture caméra) |

### Calibration Radar
- **Servo Min/Max**: Laisser 0° / 180°
- **Radar Offset**: 0° (ajuster si décalage angulaire)
- **Flip left/right**: Décocher par défaut
- **Max range**: 40 cm (pour radar court portée)

---

## 🚀 Guide d'Utilisation

### Démarrage
1. **Vérifier les connexions** (pins D6/D7, alimentation 5V)
2. **Uploader** le sketch Arduino
3. **Lancer** l'application Python
4. **Ouvrir Settings** → Cocher ✅ "Auto camera from radar"
5. **Aller à l'onglet Live**

### Fonctionnement
```
Utilisateur s'approche du capteur
        ↓
Distance ≤ ON threshold (ex: 60cm)
        ↓
Confirmation après 2 secondes
        ↓
📷 Caméra s'ACTIVE automatiquement
        ↓
[Caméra affiche le flux vidéo]
        ↓
Utilisateur s'éloigne du capteur
        ↓
Distance ≥ OFF threshold (ex: 70cm)
        ↓
Attente 10 secondes sans détection
        ↓
📷 Caméra se FERME automatiquement
```

---

## 🧪 Test de Vérification

### 1. Tester le Radar Seul
```bash
# Ouvrir l'onglet "Live"
# Vérifier que le graphique radar affiche des points
# Rapprocher/éloigner un objet → La courbe doit bouger
```

### 2. Tester l'Automatisation
```bash
# Settings: Cocher "Auto camera from radar"
# Live: Observer les seuils (60/70 cm par défaut)
# Test: Rapprocher une main → Caméra doit s'activer après ~2s
# Test: Éloigner la main → Caméra doit se fermer après ~10s d'absence
```

### 3. Dépannage
| Symptôme | Cause Probable | Solution |
|---|---|---|
| Pas de données radar | Connexion D6/D7 | Vérifier les pins Arduino |
| Caméra ne s'active pas | Pas en onglet Live | Aller à l'onglet Live |
| Caméra active mais pas d'image | Camera index incorrect | Settings → cam index |
| Mesures anormales | Problème capteur/alimentation | Vérifier 5V et GND |

---

## 📊 Données Envoyées par Arduino

L'Arduino envoie les distances au format :
```
dist:12.34
dist:12.45
dist:12.38
...
```

Format parsé automatiquement par l'application.

---

## 💡 Conseils d'Utilisation

### Performance
- Éloigner le capteur des obstacles : Meilleur signal
- Éviter les surfaces réfléchissantes proches : Peut causer des échos parasites
- Espacement des câbles : Loin des servos actifs (bruits électromagnétiques)

### Ajustements Fine-Tuning
- **Plus réactif** : Réduire "Confirm time" (ex: 1.0s)
- **Moins de faux positifs** : Augmenter "OFF threshold"
- **Fermeture plus rapide** : Réduire "Close after loss"

---

## 📝 Fichiers Modifiés

| Fichier | Modification |
|---|---|
| `arduino/face_tracker_servos.ino` | + Capteur HC-SR04, + Average mobile |
| `python/gui_app.py` | + Vérif onglet Live pour automatisation |

---

## 🔗 Ressources

- **Datasheet HC-SR04** : https://cdn.sparkfun.com/datasheets/Sensors/Ultrasonic/HCSR04.pdf
- **Arduino Ultrasonic** : https://www.arduino.cc/en/Reference/pulseIn

---

**Créé le**: 2026-05-20  
**Système**: Suivi Cible - Arduino Radar + OpenCV
