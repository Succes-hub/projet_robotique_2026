/*
 * controleur_pid.c
 * Asservissement PID pour robot e-puck
 * Projet personnel pour candidature master robotique
 * 
 * Fonctionnalités :
 * - PID pour déplacement précis
 * - Trajectoire carrée
 * - Évitement d'obstacles
 * - Export de données pour analyse
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205    // Rayon des roues de l'e-puck (en mètres)
#define AXLE_LENGTH 0.053      // Distance entre les deux roues (entraxe)

// Gains du PID (à ajuster si nécessaire)
#define KP 10.0
#define KI 0.1
#define KD 0.5

// Seuil de détection d'obstacle (capteur IR)
#define OBSTACLE_THRESHOLD 100.0

// Export des données
#define EXPORT_DATA 1
FILE *pid_data_file;

// Structure pour stocker les devices du robot
typedef struct {
    WbDeviceTag left_motor;
    WbDeviceTag right_motor;
    WbDeviceTag left_sensor;
    WbDeviceTag right_sensor;
    WbDeviceTag front_sensor;    // Capteur avant pour obstacles
} RobotDevices;

// Initialise l'export des données
void init_export_data() {
    if (EXPORT_DATA) {
        pid_data_file = fopen("pid_data.csv", "w");
        fprintf(pid_data_file, "time,error,command,distance\n");
        printf("📊 Export de données activé -> pid_data.csv\n");
    }
}

// Initialise tous les périphériques du robot
void init_devices(RobotDevices *dev) {
    // Récupération des moteurs
    dev->left_motor = wb_robot_get_device("left wheel motor");
    dev->right_motor = wb_robot_get_device("right wheel motor");
    
    // Configuration en mode vitesse (position = infini)
    wb_motor_set_position(dev->left_motor, INFINITY);
    wb_motor_set_position(dev->right_motor, INFINITY);
    
    // Arrêt initial
    wb_motor_set_velocity(dev->left_motor, 0.0);
    wb_motor_set_velocity(dev->right_motor, 0.0);
    
    // Récupération et activation des codeurs
    dev->left_sensor = wb_robot_get_device("left wheel sensor");
    dev->right_sensor = wb_robot_get_device("right wheel sensor");
    
    wb_position_sensor_enable(dev->left_sensor, TIME_STEP);
    wb_position_sensor_enable(dev->right_sensor, TIME_STEP);
    
    // Récupération et activation du capteur avant (ps0)
    dev->front_sensor = wb_robot_get_device("ps0");
    wb_distance_sensor_enable(dev->front_sensor, TIME_STEP);
    
    printf("✅ Robot e-puck initialisé avec succès en C !\n");
    printf("   - Codeurs activés\n");
    printf("   - Capteur avant activé (ps0)\n");
}

// Calcule la distance parcourue par le robot en mètres
double get_distance_traveled(RobotDevices *dev, double init_pos_left, double init_pos_right) {
    double pos_left = wb_position_sensor_get_value(dev->left_sensor);
    double pos_right = wb_position_sensor_get_value(dev->right_sensor);
    
    // Conversion radians -> mètres
    double dist_left = (pos_left - init_pos_left) * WHEEL_RADIUS;
    double dist_right = (pos_right - init_pos_right) * WHEEL_RADIUS;
    
    // La distance du robot est la moyenne des deux roues
    return (dist_left + dist_right) / 2.0;
}

// Retourne la distance à un obstacle (valeur brute du capteur)
double get_obstacle_distance(RobotDevices *dev) {
    return wb_distance_sensor_get_value(dev->front_sensor);
}

// Fonction pour avancer d'une distance précise avec PID
void move_forward_pid(RobotDevices *dev, double target_distance) {
    printf("\n>>> AVANCER: objectif = %.2f m <<<\n", target_distance);
    
    // Positions initiales des codeurs
    double init_pos_left = wb_position_sensor_get_value(dev->left_sensor);
    double init_pos_right = wb_position_sensor_get_value(dev->right_sensor);
    
    // Variables du PID
    double integral = 0.0;
    double last_error = 0.0;
    double last_time = wb_robot_get_time();
    
    int step_count = 0;
    
    // Boucle de contrôle
    while (wb_robot_step(TIME_STEP) != -1) {
        // Vérification obstacle pendant le mouvement
        if (get_obstacle_distance(dev) > OBSTACLE_THRESHOLD) {
            printf("  ⚠️ Obstacle détecté pendant le mouvement!\n");
            break;
        }
        
        // Distance actuelle
        double current_distance = get_distance_traveled(dev, init_pos_left, init_pos_right);
        
        // Calcul de l'erreur
        double error = target_distance - current_distance;
        
        // Affiche tous les 10 pas
        if (step_count % 10 == 0) {
            printf("  Distance: %.3f m | Erreur: %.3f m\n", current_distance, error);
        }
        step_count++;
        
        // Condition d'arrêt (précision 5 mm)
        if (fabs(error) < 0.005) {
            printf("  ✅ OBJECTIF ATTEINT: %.3f m\n", current_distance);
            break;
        }
        
        // Calcul du temps écoulé
        double current_time = wb_robot_get_time();
        double dt = current_time - last_time;
        
        if (dt > 0.0) {
            // Intégrale
            integral += error * dt;
            
            // Dérivée
            double derivative = (error - last_error) / dt;
            
            // Commande PID
            double command = KP * error + KI * integral + KD * derivative;
            
            // Export des données
            if (EXPORT_DATA) {
                fprintf(pid_data_file, "%f,%f,%f,%f\n", current_time, error, command, current_distance);
            }
            
            // Application aux moteurs (vitesse de base + correction)
            double base_speed = 2.0;  // rad/s
            double left_speed = base_speed + command;
            double right_speed = base_speed + command;
            
            // Limitation de vitesse
            if (left_speed > 6.28) left_speed = 6.28;
            if (left_speed < -6.28) left_speed = -6.28;
            if (right_speed > 6.28) right_speed = 6.28;
            if (right_speed < -6.28) right_speed = -6.28;
            
            wb_motor_set_velocity(dev->left_motor, left_speed);
            wb_motor_set_velocity(dev->right_motor, right_speed);
        }
        
        last_error = error;
        last_time = current_time;
    }
    
    // Arrêt des moteurs
    wb_motor_set_velocity(dev->left_motor, 0.0);
    wb_motor_set_velocity(dev->right_motor, 0.0);
    printf("  ⏹️ Moteurs arrêtés\n");
}

// Fonction pour tourner d'un angle précis
void turn_angle(RobotDevices *dev, double target_angle) {
    printf("\n>>> TOURNER: objectif = %.2f radians (%.0f degrés) <<<\n", 
           target_angle, target_angle * 180.0 / M_PI);
    
    // Positions initiales
    double init_pos_left = wb_position_sensor_get_value(dev->left_sensor);
    double init_pos_right = wb_position_sensor_get_value(dev->right_sensor);
    
    // Pour tourner, on veut une différence de distance entre les roues
    // Formule: angle = (dist_droite - dist_gauche) / entraxe
    double target_diff = target_angle * AXLE_LENGTH;
    
    double integral = 0.0;
    double last_error = 0.0;
    double last_time = wb_robot_get_time();
    
    int step_count = 0;
    
    while (wb_robot_step(TIME_STEP) != -1) {
        // Positions actuelles
        double pos_left = wb_position_sensor_get_value(dev->left_sensor);
        double pos_right = wb_position_sensor_get_value(dev->right_sensor);
        
        // Distance parcourue par chaque roue
        double dist_left = (pos_left - init_pos_left) * WHEEL_RADIUS;
        double dist_right = (pos_right - init_pos_right) * WHEEL_RADIUS;
        
        // Différence actuelle
        double current_diff = dist_right - dist_left;
        
        // Erreur
        double error = target_diff - current_diff;
        
        if (step_count % 10 == 0) {
            printf("  Différence: %.3f m | Erreur: %.3f m\n", current_diff, error);
        }
        step_count++;
        
        if (fabs(error) < 0.001) {  // 1 mm de différence ~ 0.02 rad
            printf("  ✅ ANGLE ATTEINT!\n");
            break;
        }
        
        double current_time = wb_robot_get_time();
        double dt = current_time - last_time;
        
        if (dt > 0.0) {
            integral += error * dt;
            double derivative = (error - last_error) / dt;
            
            double command = KP * error + KI * integral + KD * derivative;
            
            // Mouvement différentiel : roues en sens inverse
            double base_speed = 1.0;
            double left_speed = -base_speed - command;
            double right_speed = base_speed + command;
            
            // Limitation
            if (left_speed > 6.28) left_speed = 6.28;
            if (left_speed < -6.28) left_speed = -6.28;
            if (right_speed > 6.28) right_speed = 6.28;
            if (right_speed < -6.28) right_speed = -6.28;
            
            wb_motor_set_velocity(dev->left_motor, left_speed);
            wb_motor_set_velocity(dev->right_motor, right_speed);
        }
        
        last_error = error;
        last_time = current_time;
    }
    
    wb_motor_set_velocity(dev->left_motor, 0.0);
    wb_motor_set_velocity(dev->right_motor, 0.0);
}

// Fonction d'évitement d'obstacle
void avoid_obstacle(RobotDevices *dev) {
    printf("\n🚧 OBSTACLE DÉTECTÉ! Procédure d'évitement...\n");
    
    // 1. Reculer un peu
    printf("1. Recul...\n");
    move_forward_pid(dev, -0.2);
    
    // 2. Tourner à gauche
    printf("2. Rotation gauche...\n");
    turn_angle(dev, M_PI_2);
    
    // 3. Avancer
    printf("3. Contournement...\n");
    move_forward_pid(dev, 0.4);
    
    // 4. Tourner à droite
    printf("4. Realignement...\n");
    turn_angle(dev, -M_PI_2);
    
    // 5. Avancer pour revenir sur trajectoire
    printf("5. Retour trajectoire...\n");
    move_forward_pid(dev, 0.2);
    
    printf("✅ Obstacle évité avec succès!\n");
}

int main(int argc, char **argv) {
    // Initialisation de Webots
    wb_robot_init();
    
    printf("\n=== Projet PID Robotique ===\n");
    printf("Auteur: [TON NOM]\n");
    printf("Filière: Maths-Info\n");
    printf("Version: Améliorée (obstacle + carré + export)\n");
    printf("============================\n\n");
    
    // Initialisation des périphériques
    RobotDevices devices;
    init_devices(&devices);
    
    // Initialisation export données
    init_export_data();
    
    // Petite pause pour laisser les capteurs s'initialiser
    int i;
    for (i = 0; i < 10; i++) {
        wb_robot_step(TIME_STEP);
        printf("Initialisation capteurs... %d/10\r", i+1);
        fflush(stdout);
    }
    printf("\n");
    
    // --- MISSION PRINCIPALE : TRAJECTOIRE CARRÉE AVEC ÉVITEMENT ---
    printf("\n🚀 Début de la mission : Carré de 1m avec gestion d'obstacles\n");
    
    // Vérifier obstacle au départ
    double obstacle_dist = get_obstacle_distance(&devices);
    printf("Distance obstacle initiale: %.0f\n", obstacle_dist);
    
    if (obstacle_dist > OBSTACLE_THRESHOLD) {
        avoid_obstacle(&devices);
    }
    
    // Réalisation du carré
    for (int cote = 0; cote < 4; cote++) {
        printf("\n═══════════════════════════════\n");
        printf("   CÔTÉ %d/4 DU CARRÉ\n", cote+1);
        printf("═══════════════════════════════\n");
        
        // Vérifier obstacle avant d'avancer
        if (get_obstacle_distance(&devices) > OBSTACLE_THRESHOLD) {
            avoid_obstacle(&devices);
        }
        
        // Avancer d'1 mètre
        move_forward_pid(&devices, 1.0);
        
        // Petite pause entre les mouvements
        for (i = 0; i < 10; i++) wb_robot_step(TIME_STEP);
        
        // Tourner pour le côté suivant (sauf après le dernier)
        if (cote < 3) {
            turn_angle(&devices, M_PI_2);
            for (i = 0; i < 10; i++) wb_robot_step(TIME_STEP);
        }
    }
    
    printf("\n═══════════════════════════════\n");
    printf("🏆 MISSION TERMINÉE AVEC SUCCÈS!\n");
    printf("═══════════════════════════════\n");
    
    // Fermeture du fichier de données
    if (EXPORT_DATA) {
        fclose(pid_data_file);
        printf("\n📊 Données exportées dans pid_data.csv\n");
        printf("   Utilise Excel ou Python pour générer un graphique\n");
    }
    
    // Statistiques finales
    printf("\n📈 Statistiques:\n");
    printf("   - Gains PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", KP, KI, KD);
    printf("   - Seuil obstacle: %.0f\n", OBSTACLE_THRESHOLD);
    printf("   - Temps de simulation: %.2f s\n", wb_robot_get_time());
    
    // Boucle infinie pour éviter que le programme se termine
    printf("\n⏸️ Simulation terminée. Ferme la fenêtre pour quitter.\n");
    while (wb_robot_step(TIME_STEP) != -1) {
        // Ne rien faire, juste attendre
    }
    
    wb_robot_cleanup();
    
    return 0;
} 