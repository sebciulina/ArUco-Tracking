#include <chrono>
#include <iostream>
#include <list>
#include <random>
#include <tuple>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// ==========================================
//               CONFIGURATION
// ==========================================

// Change this value to match the real-world printed size of your ArUco marker (in millimeters)
constexpr double MARKER_SIZE_MM = 30.0;

// Camera device ID (0 for built-in, 1 for external USB camera)
constexpr int CAMERA_ID = 1;

// Relative paths to assets (Ensure the 'assets' folder is in your working directory)
const std::string CAR1_IMG_PATH = "assets/car0.png";
const std::string CAR2_IMG_PATH = "assets/car1.png";
const std::string OBSTACLE_IMG_PATH = "assets/obstacle.png";

constexpr int WINNING_SCORE = 5;

// ==========================================

std::random_device rd;
std::mt19937 generator(rd());

std::pair<cv::Mat, cv::Mat> get_camera_properties_ps3eye()
{
    // Coefficients for approximated PS3 Eye camera
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 600, 0, 320,
                                                       0, 600, 240,
                                                       0,   0,   1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);

    return {camera_matrix, dist_coeffs};
}

cv::Mat get_object_points()
{
    cv::Mat obj_points(4, 1, CV_32FC3);
    obj_points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-MARKER_SIZE_MM / 2.f, MARKER_SIZE_MM / 2.f, 0);
    obj_points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(MARKER_SIZE_MM / 2.f, MARKER_SIZE_MM / 2.f, 0);
    obj_points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(MARKER_SIZE_MM / 2.f, -MARKER_SIZE_MM / 2.f, 0);
    obj_points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-MARKER_SIZE_MM / 2.f, -MARKER_SIZE_MM / 2.f, 0);
    return obj_points;
}

int main()
{
    using namespace cv;
    using namespace std;

    // 1. Load assets with error checking
    Mat car_image = imread(CAR1_IMG_PATH);
    Mat car_image2 = imread(CAR2_IMG_PATH);
    Mat goal_image = imread(OBSTACLE_IMG_PATH);

    if (car_image.empty() || car_image2.empty() || goal_image.empty()) {
        cerr << "[ERROR] Could not load images. Make sure the 'assets' folder exists and contains car0.png, car1.png, and obstacle.png." << endl;
        return -1;
    }

    // 2. Initialize camera
    VideoCapture cam(CAMERA_ID);
    if (!cam.isOpened()) {
        cerr << "[ERROR] Could not open camera with ID: " << CAMERA_ID << endl;
        return -1;
    }

    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    namedWindow("markers", WINDOW_NORMAL);

    auto [camera_matrix, dist_coeffs] = get_camera_properties_ps3eye();
    auto obj_points = get_object_points();

    Vec2f car_position(200, 200);
    Vec2f car_position2(420, 200);

    vector<Vec2f> goals;
    int counter = 0;
    int points = 0;
    int points2 = 0;

    // Main interaction loop
    while (waitKey(10) != 27) { // Press 'ESC' to exit
        Mat inputImage;
        cam >> inputImage;
        if (inputImage.empty()) break;
        
        Mat detected = inputImage.clone();

        // Spawn obstacles periodically
        if ((counter++ % 20 == 0) && (goals.size() < 10)) {
            std::uniform_int_distribution<int> distr_x(goal_image.cols, detected.cols - goal_image.cols);
            std::uniform_int_distribution<int> distr_y(goal_image.rows, detected.rows - goal_image.rows);
            Vec2f p(distr_x(generator), distr_y(generator));
            goals.push_back(p);
        }

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners, rejectedCandidates;
        
        // Detect ArUco markers
        aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        aruco::drawDetectedMarkers(detected, markerCorners, markerIds);

        // --- REFACTOR: Lambda function to handle pose estimation and collision for any vehicle ---
        auto process_vehicle = [&](int target_id, Vec2f& pos, Mat& img, int& score) {
            auto found = find(markerIds.begin(), markerIds.end(), target_id);
            if (found != markerIds.end()) {
                int foundIdx = distance(markerIds.begin(), found);
                auto corners = markerCorners.at(foundIdx);

                cv::Vec3d rvecs, tvecs;
                
                // Calculate 3D pose
                cv::solvePnP(obj_points, corners, camera_matrix, dist_coeffs, rvecs, tvecs);
                cv::drawFrameAxes(detected, camera_matrix, dist_coeffs, rvecs, tvecs, MARKER_SIZE_MM / 2.0);

                // Convert rotation vector to rotation matrix
                Mat rot_mat;
                Rodrigues(rvecs, rot_mat);
                
                // Extract Euler angles
                double angle_x = atan2(rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
                angle_x = -angle_x + ((angle_x < 0) ? (-CV_PI) : CV_PI);
                double angle_y = -asin(rot_mat.at<double>(2, 0));

                // Update car position based on marker rotation
                auto new_pos = pos;
                new_pos.val[0] += 10.0 * angle_y;
                new_pos.val[1] -= 10.0 * angle_x;
                
                // Boundary collision check
                auto new_pos_scr = new_pos;
                new_pos_scr.val[0] -= img.cols / 2.0f;
                new_pos_scr.val[1] -= img.rows / 2.0f;
                
                if ((new_pos_scr.val[0] > 0) && (new_pos_scr.val[0] < (detected.cols - img.cols - 1)) && 
                    (new_pos_scr.val[1] > 0) && (new_pos_scr.val[1] < (detected.rows - img.rows - 1))) {
                    pos = new_pos;
                }
            }

            // Safely draw car inside bounds
            Rect car_rect(pos.val[0] - (img.cols / 2), pos.val[1] - (img.rows / 2), img.cols, img.rows);
            if (car_rect.x >= 0 && car_rect.y >= 0 && car_rect.x + car_rect.width < detected.cols && car_rect.y + car_rect.height < detected.rows) {
                Mat insetImage(detected, car_rect);
                img.copyTo(insetImage);
            }

            // Goal collision logic using Euclidean distance
            for (int i = 0; i < goals.size(); i++) {
                if (cv::norm(goals[i] - pos) < (goal_image.cols / 2.0)) {
                    score++;
                    goals.erase(goals.begin() + i);
                    i--;
                }
            }
        };
        // -----------------------------------------------------------------------------------------

        // Process Player 1 (Red Car - ID 0)
        process_vehicle(0, car_position, car_image, points);
        
        // Process Player 2 (Green Car - ID 1)
        process_vehicle(1, car_position2, car_image2, points2);

        // Draw goals safely
        for (auto goal : goals) {
            Rect goal_rect(goal.val[0] - (goal_image.cols / 2), goal.val[1] - (goal_image.rows / 2), goal_image.cols, goal_image.rows);
            if (goal_rect.x >= 0 && goal_rect.y >= 0 && goal_rect.x + goal_rect.width < detected.cols && goal_rect.y + goal_rect.height < detected.rows) {
                Mat insetImage(detected, goal_rect);
                goal_image.copyTo(insetImage);
            }
        }

        // Display UI
        putText(detected, "P1 Score: " + to_string(points), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        putText(detected, "P2 Score: " + to_string(points2), Point(10, detected.rows - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
        imshow("markers", detected);

        if (points >= WINNING_SCORE || points2 >= WINNING_SCORE) {
            break;
        }
    }

    if (points > points2) {
        cout << "Player 1 (Red car) wins with " << points << " points!" << endl;
    } else if (points < points2) {
        cout << "Player 2 (Green car) wins with " << points2 << " points!" << endl;
    } else {
        cout << "There is a tie!" << endl;
    }
    
    return 0;
}