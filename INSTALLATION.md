# Installation du Projet

## 1. Prérequis
- Python 3.8+ installé
- pip (gestionnaire de paquets Python)

## 2. Créer un environnement virtuel (Optionnel mais recommandé)

```bash
python -m venv .venv
```

## 3. Activer l'environnement virtuel

### Sur Windows (PowerShell):
```bash
.\.venv\Scripts\Activate.ps1
```

### Sur Windows (CMD):
```bash
.\.venv\Scripts\activate.bat
```

### Sur macOS/Linux:
```bash
source .venv/bin/activate
```

## 4. Installer toutes les dépendances

Une fois dans l'environnement virtuel, exécutez:

```bash
pip install -r requirements.txt
```

## 5. Dépendances principales nécessaires:

✅ **PyQt6** - Interface graphique  
✅ **opencv-python** - Traitement vidéo  
✅ **ultralytics** - Détection d'objets YOLO  
✅ **pyserial** - Communication avec Arduino  
✅ **torch** - Framework pour le machine learning (nécessaire pour ultralytics)  

## 6. Modèles YOLO

Le code assume que vous avez un des fichiers modèles YOLO:
- `yolov8n.pt` (nano - léger, rapide)
- `yolov8m.pt` (medium - équilibre)

Ces fichiers seront téléchargés automatiquement à la première exécution.

## 7. Lancer l'application

```bash
python python/gui_app.py
```

## ⚠️ Problèmes courants

**Erreur "ModuleNotFoundError"**: Assurez-vous que:
1. L'environnement virtuel est activé
2. Vous avez exécuté `pip install -r requirements.txt`

**Erreur avec la caméra**: Vérifiez que:
1. Votre webcam est connectée
2. Aucune autre application n'utilise la caméra

**Erreur série Arduino**: Vérifiez que:
1. L'Arduino est connecté au port USB
2. Le port série est correctement détecté
