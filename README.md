# 🤖 Projet Robotique : Asservissement PID d'un robot e-puck

## 🎯 Objectif
Ce projet démontre la maîtrise du **contrôle PID** (Proportionnel-Intégral-Dérivé) pour un déplacement précis en robotique. Développé sous **Webots** en C.

## 🛠️ Fonctionnalités
- ✅ Asservissement PID pour un déplacement linéaire précis (erreur < 5mm)
- ✅ Rotation précise à 90°
- ✅ Trajectoire carrée autonome    
- ✅ Évitement d'obstacles par capteur infrarouge
- ✅ Export de données pour analyse

## 🎥 Démonstration
[![Vidéo du projet](lien_vers_ton_image)](lien_vers_ta_video)
*Cliquez sur l'image pour voir la vidéo*

## 📐 Mathématiques
- **PID** : `u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt`
- **Odométrie** : `distance = (Δg + Δd)/2 × rayon_roue`
- **Gains utilisés** : Kp=10.0, Ki=0.1, Kd=0.5

## 📊 Résultats
![Graphique PID](pid_graph.png)
*Convergence de l'erreur vers 0*

## 🚀 Utilisation
1. Installer [Webots](https://cyberbotics.com/)
2. Ouvrir le monde : `worlds/mon_projet.wbt`
3. Sélectionner le controller : `test_pid`
4. Lancer la simulation

## 📁 Structure du projet