#include "Simulation.h"

void Simulation::calibration()
{
    Settings s;
    const string inputSettingsFile = parser.get<string>(0);
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        parser.printMessage();
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    for (;;)
    {
        Mat view;
        bool blinkOutput = false;

        view = s.nextImage();

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if (mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames)
        {
            if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, grid_width,
                release_object))
                mode = CALIBRATED;
            else
                mode = DETECTION;
        }
        if (view.empty())          // If there are no more images stop the loop
        {
            // if calibration threshold was not reached yet, calibrate now
            if (mode != CALIBRATED && !imagePoints.empty())
                runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, grid_width,
                    release_object);
            break;
        }
        vector<Point2f> pointBuf;

        bool found;

        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        if (!s.useFisheye) {
            // fast check erroneously fails with high distortions like fisheye
            chessBoardFlags |= CALIB_CB_FAST_CHECK;
        }

        switch (s.calibrationPattern) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
            break;
        case Settings::CHARUCOBOARD:
            ch_detector.detectBoard(view, pointBuf, markerIds);
            found = pointBuf.size() == (size_t)((s.boardSize.height - 1) * (s.boardSize.width - 1));
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid(view, s.boardSize, pointBuf);
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            found = false;
            break;
        }
        if (found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if (s.calibrationPattern == Settings::CHESSBOARD)
            {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, Size(winSize, winSize),
                    Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
            }

            if (mode == CAPTURING &&  // For camera only take new samples after delay time
                (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay * 1e-3 * CLOCKS_PER_SEC))
            {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
                blinkOutput = s.inputCapture.isOpened();
            }

            // Draw the corners.
            if (s.calibrationPattern == Settings::CHARUCOBOARD)
                drawChessboardCorners(view, cv::Size(s.boardSize.width - 1, s.boardSize.height - 1), Mat(pointBuf), found);
            else
                drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
        }
        string msg = (mode == CAPTURING) ? "100/100" :
            mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

        if (mode == CAPTURING)
        {
            if (s.showUndistorted)
                msg = cv::format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
            else
                msg = cv::format("%d/%d", (int)imagePoints.size(), s.nrFrames);
        }

        putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

        if (blinkOutput)
            bitwise_not(view, view);
        if (mode == CALIBRATED && s.showUndistorted)
        {
            Mat temp = view.clone();
            if (s.useFisheye)
            {
                Mat newCamMat;
                fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                    Matx33d::eye(), newCamMat, 1);
                cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs, newCamMat);
            }
            else
                undistort(temp, view, cameraMatrix, distCoeffs);
        }
        imshow("Image View", view);
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if (key == ESC_KEY)
            break;

        if (key == 'u' && mode == CALIBRATED)
            s.showUndistorted = !s.showUndistorted;

        if (s.inputCapture.isOpened() && key == 'g')
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
        if (s.inputType == Settings::IMAGE_LIST && s.showUndistorted && !cameraMatrix.empty())
        {
            Mat view, rview, map1, map2;

            if (s.useFisheye)
            {
                Mat newCamMat;
                fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                    Matx33d::eye(), newCamMat, 1);
                fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
                    CV_16SC2, map1, map2);
            }
            else
            {
                initUndistortRectifyMap(
                    cameraMatrix, distCoeffs, Mat(),
                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
                    CV_16SC2, map1, map2);
            }

            for (size_t i = 0; i < s.imageList.size(); i++)
            {
                view = imread(s.imageList[i], IMREAD_COLOR);
                if (view.empty())
                    continue;
                remap(view, rview, map1, map2, INTER_LINEAR);
                imshow("Image View", rview);
                char c = (char)waitKey();
                if (c == ESC_KEY || c == 'q' || c == 'Q')
                    break;
            }
        }
        bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
            vector<vector<Point2f> > imagePoints, float grid_width, bool release_object)
        {
            vector<Mat> rvecs, tvecs;
            vector<float> reprojErrs;
            double totalAvgErr = 0;
            vector<Point3f> newObjPoints;

            bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                totalAvgErr, newObjPoints, grid_width, release_object);
            cout << (ok ? "Calibration succeeded" : "Calibration failed")
                << ". avg re projection error = " << totalAvgErr << endl;

            if (ok)
                saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                    totalAvgErr, newObjPoints);
            return ok;
        }
        vector<vector<Point3f> > objectPoints(1);
        calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
        objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
        newObjPoints = objectPoints[0];

        objectPoints.resize(imagePoints.size(), objectPoints[0]);

}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
    Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch (patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; ++i) {
            for (int j = 0; j < boardSize.width; ++j) {
                corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
            }
        }
        break;
    case Settings::CHARUCOBOARD:
        for (int i = 0; i < boardSize.height - 1; ++i) {
            for (int j = 0; j < boardSize.width - 1; ++j) {
                corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
            }
        }
        break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++) {
            for (int j = 0; j < boardSize.width; j++) {
                corners.push_back(Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
            }
        }
        break;
    default:
        break;
    }
}

int Simulation::predict(BallInfo ballInfo)
{
    double BallPos[2];
    double BallSpeed[2];
    BallPos[0] = ballInfo.BallXPosition;
    BallPos[1] = ballInfo.BallYPosition;
    BallSpeed[0] = ballInfo.BallXVelocity;
    BallSpeed[1] = ballInfo.BallYVelocity;
// POINT 1 AND 2 ARE UP RIGHT AND DOWN RIGHT
    std::array<float, 2> rightmost{};
    rightmost[0]=corners[1];
    rightmost[1]=corners[2];
    int remainingWidth = BallPos[0]+rightmost2.x;
    int multiplier = remainingWidth/BallSpeed[0];
    if (BallSpeed[0]<0){ 
        multiplier=-multiplier;
    }
    int predictedY=BallPos[1]+multiplier*BallSpeed[1];
    if (predictedY>Rightmost[0].y){
        predictedY= Rightmost[0].y;
    }
    else if (predictedY<Rightmost[1].y){
        predictedY =Rightmost[1].y;
    }
    return predictedY;
}

cv::Point Simulation::simulateStep(BallInfo info)
{
	return cv::Point();
}
	