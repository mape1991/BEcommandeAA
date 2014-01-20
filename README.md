#BE Commande - Automatique avancée 
==================================

###Régulation d’une turbine à gaz

##Introduction

##Modélisation (Modèle de vérification)
0. Système non-linéaire avec tables des valeurs
1. Doseur carburant
2. Arbre générateur
3. Turbine libre

##Linearisation (Simplification avec un modèle de synthèse)
0. Calcul du point d'équilibre
1. Linéarisation du modèle 

##Synthèse des contrôleurs (Satisfaction du Cahier des charges)
###Observateur de Kalman
0. Equations du filtre de Kalman
1. Incertitudes sur les données
2. Calcul du gain du filtre

###Commande par retour d’état
0. Performances du système linéarisé
1. Cahier des charges pour la commande
2. Placement des pôles
3. Retour d'état augmenté
4. Pire echantilon de charge (Pire scenario de test)

###Commande robuste (LMI)
0. Calcul du retour d'état K_lmi stabilisant
1. Vérification de la stabilité du système 

##Validation et améliorations
0. Associer le filtre de Kalman avec le retour d'état
1. Ajouter une commande H_infini
2. Commande LQR
