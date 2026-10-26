#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "robot.hpp"
#include "lidar.hpp"
#include "exploration.hpp"
#include "wallfollow.hpp"
#include "keyboard.hpp"
#include "behavior.hpp"
#include "aruco_manager.hpp"
#include "camera.hpp"
#include "occupancy_grid.hpp"

namespace {
const char* modeName(CommandMode m) {
    switch (m) {
        case CommandMode::MANUAL:      return "MANUAL";
        case CommandMode::EXPLORATION: return "EXPLORATION";
        default:                       return "WALLFOLLOW";
    }
}
}

int main() {

    // ============ INITIALISATION ============

    // chemin relatif au dossier build/ ou à la racine du projet
    LiDAR lidar {"../res/img.png"};
    if (lidar.getMap().empty()) lidar = LiDAR{"res/img.png"};
    cv::Mat carte_principale = lidar.getMap();
    if (carte_principale.empty()) {
        std::cerr << "Erreur : carte res/img.png introuvable "
                     "(lancer depuis build/ ou depuis la racine du projet)." << std::endl;
        return 1;
    }

    const int LARGEUR_CARTE = carte_principale.cols;
    const int HAUTEUR_CARTE = carte_principale.rows;
    const int TAILLE_ROBOT = 10;
    const float DELTA_TEMPS = 0.033f;

    float initX = 100;
    float initY = 670;
    Robot robot {initX, initY};

    OccupancyGrid grid {LARGEUR_CARTE, HAUTEUR_CARTE};
    Exploration exploration(&robot, &lidar, &grid);
    WallFollow wall(&lidar);
    Keyboard keyboard;
    BehaviorManager behavior;

    Camera camera;
    cv::VideoCapture& cap = camera.getCapture();
    if (!camera.isOpened())
        std::cout << "[Camera] Aucune caméra : modes pilotés au clavier uniquement (E/F/M)." << std::endl;

    ArucoManager aruco;

    // carte en couleur pour pouvoir dessiner le robot en rouge
    cv::Mat carte_couleur, carte_actuelle;
    cv::cvtColor(carte_principale, carte_couleur, cv::COLOR_GRAY2BGR);
    const cv::Scalar couleur_robot(0, 0, 255); // Rouge (BGR)

    cv::namedWindow("Map du Robot", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Aruco detector", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Occupancy Grid", cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    int id = -1;
    int lastArucoSeen = -1;   // pour ne réagir qu'à l'apparition d'un marqueur

    std::cout << "=== SIMULATION ROBOT ===" << std::endl;
    std::cout << "Commandes clavier:" << std::endl;
    std::cout << "  E - Mode EXPLORATION" << std::endl;
    std::cout << "  F - Mode WALLFOLLOW" << std::endl;
    std::cout << "  M - Mode MANUAL (W/A/S/D pour bouger, R pour revenir au départ)" << std::endl;
    std::cout << "  Q / Echap - Quitter" << std::endl;
    std::cout << "ArUco (6x6_250) : ID 0 = manuel, 1 = exploration, 42 = wallfollow" << std::endl;
    std::cout << "========================" << std::endl;

    // ============ BOUCLE PRINCIPALE ============

    while (true)
    {
        // ---- Caméra / ArUco ----
        if (camera.isOpened()) cap.read(frame);
        if (frame.empty()) {   // pas de caméra : image noire pour l'affichage
            frame = cv::Mat::zeros(480, 640, CV_8UC3);
            cv::putText(frame, "Pas de camera", {10, 450},
                        cv::FONT_HERSHEY_SIMPLEX, .6, {200, 200, 200}, 1);
        }

        // detectAruco ne renvoie true que si un marqueur est réellement vu.
        // On ne change de mode qu'à l'APPARITION d'un marqueur (sinon l'ancien ID
        // écraserait en permanence les touches clavier).
        if (aruco.detectAruco(id, frame)) {
            if (id != lastArucoSeen) {
                if (behavior.setBehavior(id))
                    std::cout << "[ArUco] ID=" << id << " -> mode " << modeName(behavior.currentMode()) << std::endl;
                lastArucoSeen = id;
            }
        } else {
            lastArucoSeen = -1;
        }

        // ---- Touches ----
        int key = cv::waitKey(1);
        if (key != -1) key &= 0xFF;

        if (key == 27 || key == 'Q' || key == 'q')
            break;

        CommandMode before = behavior.currentMode();
        if (key == 'e' || key == 'E')      behavior.setBehavior(1);
        else if (key == 'f' || key == 'F') behavior.setBehavior(42);
        else if (key == 'm' || key == 'M') behavior.setBehavior(0);

        if (behavior.currentMode() != before) {
            robot.setVx(0);          // pas de vitesse résiduelle d'un autre mode
            robot.setVtheta(0);
            std::cout << "[Mode] " << modeName(behavior.currentMode()) << std::endl;
        }

        // ---- Commande + déplacement ----
        const float oldX = robot.getX(), oldY = robot.getY();

        if (behavior.currentMode() == CommandMode::MANUAL) {
            if (key != -1 && key != 255)
                keyboard.processKey(key, robot, initX, initY);
        }
        else if (behavior.currentMode() == CommandMode::EXPLORATION) {
            exploration.explorer(robot);
            robot.deplacer(DELTA_TEMPS);
        }
        else {
            wall.suivre(robot);
            robot.deplacer(DELTA_TEMPS);
        }

        // ---- Collision : le robot ne traverse pas les murs / ne sort pas de la carte ----
        {
            int rx = static_cast<int>(std::lround(robot.getX()));
            int ry = static_cast<int>(std::lround(robot.getY()));
            bool hors = rx < 0 || ry < 0 || rx >= LARGEUR_CARTE || ry >= HAUTEUR_CARTE;
            if (hors || carte_principale.at<uchar>(ry, rx) < 50) {
                robot.setX(oldX);
                robot.setY(oldY);
            }
        }

        // ---- Perception : scan APRES le déplacement (position à jour) ----
        std::vector<float> distances =
            lidar.scan(robot.getX(), robot.getY(), robot.getHeadAngleDeg());
        grid.updateFromScan(robot.getX(), robot.getY(), robot.getHeadAngleDeg(), distances);

        // ---- Affichage mode sur l'image caméra ----
        cv::Scalar couleurMode = behavior.currentMode() == CommandMode::MANUAL      ? cv::Scalar(255, 165, 0)
                               : behavior.currentMode() == CommandMode::EXPLORATION ? cv::Scalar(0, 255, 0)
                                                                                    : cv::Scalar(255, 0, 0);
        cv::putText(frame, std::string(modeName(behavior.currentMode())) + " MODE", {10, 50},
                    cv::FONT_HERSHEY_COMPLEX, .7, couleurMode, 2);

        // ---- Affichage carte ----
        carte_couleur.copyTo(carte_actuelle);

        cv::Point centre(static_cast<int>(robot.getX()), static_cast<int>(robot.getY()));
        cv::circle(carte_actuelle, centre, TAILLE_ROBOT, couleur_robot, cv::FILLED);

        float angle_rad = robot.getHeadAngleDeg() * static_cast<float>(M_PI) / 180.0f;
        cv::Point fleche(static_cast<int>(robot.getX() + TAILLE_ROBOT * 1.5f * std::cos(angle_rad)),
                         static_cast<int>(robot.getY() + TAILLE_ROBOT * 1.5f * std::sin(angle_rad)));
        cv::line(carte_actuelle, centre, fleche, cv::Scalar(255, 0, 0), 2);

        cv::imshow("Map du Robot", carte_actuelle);
        cv::imshow("Aruco detector", frame);
        cv::imshow("Occupancy Grid", grid.getGrid());
    }

    std::cout << "\n=== FIN DE LA SIMULATION ===" << std::endl;
    return 0;
}
