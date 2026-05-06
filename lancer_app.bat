@echo off
:: Lanceur Suivi Cible — double-clic ou executer depuis le terminal
cd /d "%~dp0python"
python gui_app.py
if errorlevel 1 (
    echo.
    echo ERREUR : l'application a plante. Appuie sur une touche pour voir le detail.
    pause
)
