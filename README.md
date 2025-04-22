# tp_turtle_regulation_suhayl_yann
# TP Turtle Regulation

Ce projet implémente un système de régulation pour la tortue turtlesim de ROS2. Le système permet de définir un point de destination (waypoint) et fait naviguer la tortue jusqu'à ce point en utilisant des contrôleurs proportionnels pour la régulation en cap et en distance.

## Packages

Le projet contient deux packages :

1. `turtle_regulation` : Contient les nœuds pour la régulation de la tortue
2. `turtle_interfaces` : Contient les définitions de services pour les interactions entre nœuds

## Installation

### Prérequis

- ROS2 Jazzy
- Python 3.8+

### Étapes d'installation

1. Clonez ce dépôt dans votre workspace ROS2 :
```bash
cd ~/ros2_ws/src
git clone https://github.com/VOTRE_USERNAME/tp_turtle_regulation_suhayl_yann.git
