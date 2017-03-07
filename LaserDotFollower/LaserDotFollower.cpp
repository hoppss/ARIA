/*
 * This program is a testbed for testing various vision algorithms that enable a Pioneer P3-DX
 * robot to follow a laser dot, as produced by a standard laser pointer.
 *
 * Joost van Stuijvenberg
 * Avans Hogeschool Breda
 * March 2017
 *
 * CC BY-SA 4.0, see: https://creativecommons.org/licenses/by-sa/4.0/
 * sources & updates: https://github.com/joostvanstuijvenberg/ARIA
 *
 * You are free to:
 *    Share — copy and redistribute the material in any medium or format
 *    Adapt — remix, transform, and build upon the material for any purpose, even commercially.
 *
 * The licensor cannot revoke these freedoms as long as you follow the license terms.
 *
 * Under the following terms:
 *    Attribution — You must give appropriate credit, provide a link to the license, and indicate
 *                  if changes were made. You may do so in any reasonable manner, but not in any
 * 	                way that suggests the licensor endorses you or your use.
 *    ShareAlike  — If you remix, transform, or build upon the material, you must distribute your
 *                  contributions under the same license as the original.
 *
 * No additional restrictions — You may not apply legal terms or technological measures that
 * legally restrict others from doing anything the license permits.
 *
 * Notices:
 *    You do not have to comply with the license for elements of the material in the public domain
 *    or where your use is permitted by an applicable exception or limitation. No warranties are
 *    given. The license may not give you all of the permissions necessary for your intended use.
 *    For example, other rights such as publicity, privacy, or moral rights may limit how you use
 *    the material.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Aria.h"

 // Suppress the console window (in Visual Studio).
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

#define CAMERA_NUMBER				1

#define ROBOT_MAX_DRIVING_SPEED		50.0 //500
#define ROBOT_MAX_ROTATING_SPEED	25.0 //100

cv::Ptr<cv::SimpleBlobDetector> getBlobDetector();
cv::Point findLaserDotByRGB(cv::Mat& image);
cv::Point findLaserDotByHSV(cv::Mat& image);
cv::Point findLaserDotByHSVOtsu(cv::Mat& image);
cv::Point findLaserDotByBlobDetection(cv::Mat& image, cv::Ptr<cv::SimpleBlobDetector>& detector);
void initRobot(ArRobot& robot, int argc, char** argv);
void controlRobot(ArRobot& robot, double deltaX, double deltaY);
void shutdownRobot(ArRobot& robot);

/*
 * ---------------------------------------------------------------------------------------------- *
 * main()                                                                                         *
 * ---------------------------------------------------------------------------------------------- *
 */
int main(int argc, char** argv)
{
	ArRobot robot;
	bool robotEnabled = true;
	cv::Ptr<cv::SimpleBlobDetector> detector;

	// Try to open the camera.
	cv::VideoCapture camera(CAMERA_NUMBER);
	if (!camera.isOpened())
	{
		std::cout << "Could not open camera " << CAMERA_NUMBER << std::endl;
		std::cin.get();
		return -1;
	}

	// Connect to the robot, when appropriate.
	if (robotEnabled)
		initRobot(robot, argc, argv);

	detector = getBlobDetector();

	// Repeatedly obtain an image from the camera and try to find the red laser dot.
	cv::Mat image;
	cv::Point point;
	char key = 0;
	bool halt = false;
	while (key != 27)
	{
		if (key == 32)
			halt = !halt;

		// Obtain an image from the camera.
		camera >> image;
		if (image.empty())
		{
			std::cout << "Camera stream ended unexpectedly" << std::endl;
			std::cin.get();
			break;
		}

		cv::flip(image, image, 0);
		//point = findLaserDotByBlobDetection(image, detector);
		//point = findLaserDotByHSV(image);
		//point = findLaserDotByHSVOtsu(image);
		point = findLaserDotByRGB(image);
		if (point.x != -1 && point.y != -1)
		{
			double halfWidth = image.size().width / 2.0;
			double halfHeight = image.size().height / 2.0;
			double deltaX = (halfWidth - point.x) / halfWidth;
			double deltaY = (halfHeight - point.y) / halfHeight;
			if (abs(deltaX) < 0.1 && abs(deltaY) < 0.1)
			{
				deltaX = 0.0;
				deltaY = 0.0;
			}
			std::cout << "Delta is " << deltaX << ',' << deltaY << std::endl;
			if (robotEnabled)
				controlRobot(robot, deltaX, deltaY);
		}

		// Wait 40 msec (if not halted), thus obtaining a theoretical fps of 25.
		key = cv::waitKey(halt == true ? 0 : 40);
	}

	// Shut down the robot, when appropriate.
	if (robotEnabled)
		shutdownRobot(robot);
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * getBlobDetector()                                                                              *
 * ---------------------------------------------------------------------------------------------- *
 */
cv::Ptr<cv::SimpleBlobDetector> getBlobDetector()
{
	// Setup a SimpleBlobDetector for detection of the laser dot.
	cv::SimpleBlobDetector::Params params;
	params.minThreshold = 50;
	params.maxThreshold = 255;
	params.thresholdStep = 5;
	params.filterByArea = true;
	params.minArea = 1;
	params.maxArea = 20;
	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByCircularity = false;
	//params.minCircularity = 0.8;
	//params.maxCircularity = 1.0;
	params.filterByConvexity = false;
	params.filterByInertia = false;
	cv::Ptr<cv::SimpleBlobDetector> result = cv::SimpleBlobDetector::create(params);
	return result;
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * findLaserDotByRGB()                                                                            *
 * ---------------------------------------------------------------------------------------------- *
 */
cv::Point findLaserDotByRGB(cv::Mat& image)
{
	cv::Mat dot;
	std::vector<std::vector<cv::Point>> contours;

	// Show the original image. Adjust camera aperture to return a reasonably dark image.
	cv::imshow("Source", image);
	cv::moveWindow("Source", 500, 0);

	// Allow only red or 'reddish' dots to pass through, using inRange (i.e. 3 channel threshold).
	// Then dilate the binary image for enhanced visibility.
	cv::inRange(image, cv::Scalar(0, 0, 225), cv::Scalar(25, 25, 255), dot);
	cv::dilate(dot, dot, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
	cv::imshow("Dot", dot);
	cv::moveWindow("Dot", 1000, 500);

	// Return (-1, -1) when more than one blob was found. Otherwise, see if the x and y delta are
	// within limits to return (0, 0). In all other cases, return the 
	cv::findContours(dot, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	if (contours.size() == 1)
		return contours.at(0).at(0);

	return cv::Point(-1, -1);
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * findLaserDotByHSV()                                                                            *
 * ---------------------------------------------------------------------------------------------- *
 */
cv::Point findLaserDotByHSV(cv::Mat& image)
{
	cv::Mat hsv;
	cv::Mat channels[3], dot1, dot2, dot;

	cv::imshow("Source", image);
	cv::moveWindow("Source", 1000, 0);

	cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	cv::split(hsv, channels);

	cv::inRange(hsv, cv::Scalar(0, 210, 210), cv::Scalar(15, 255, 255), dot1);
	cv::dilate(dot1, dot1, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
	cv::imshow("Dot1", dot1);
	cv::moveWindow("Dot1", 500, 300);

	cv::inRange(hsv, cv::Scalar(165, 230, 230), cv::Scalar(180, 255, 255), dot2);
	cv::dilate(dot2, dot2, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
	cv::imshow("Dot2", dot2);
	cv::moveWindow("Dot2", 1000, 300);

	cv::bitwise_or(dot1, dot2, dot);
	cv::imshow("Dot", dot);
	cv::moveWindow("Dot", 1000, 600);

	for (int y = 0; y < dot.rows; y++)
		for (int x = 0; x < dot.cols; x++)
			if (dot.at<uchar>(y, x) == 255)
				return cv::Point2d(x, y);

	return cv::Point(-1, -1);
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * findLaserDotByHSVOtsu()                                                                        *
 * ---------------------------------------------------------------------------------------------- *
 */
cv::Point findLaserDotByHSVOtsu(cv::Mat& image)
{
	cv::Mat hsv, channels[3], dot;

	cv::imshow("Source", image);
	cv::moveWindow("Source", 500, 0);

	cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	cv::split(hsv, channels);

	cv::imshow("Saturation", channels[1]);
	cv::moveWindow("Saturation", 700, 300);

	cv::threshold(channels[1], dot, 225, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	cv::imshow("Dot", dot);
	cv::moveWindow("Dot", 900, 600);

	for (int y = 0; y < dot.rows; y++)
		for (int x = 0; x < dot.cols; x++)
			if (dot.at<uchar>(y, x) == 255)
				return cv::Point(x, y);

	return cv::Point(-1, -1);
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * findLaserDotByBlobDetection()                                                                  *
 * ---------------------------------------------------------------------------------------------- *
 */
cv::Point findLaserDotByBlobDetection(cv::Mat& image, cv::Ptr<cv::SimpleBlobDetector>& detector)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat gray, blob;
	cv::Point result;

	cv::imshow("Source", image);
	cv::moveWindow("Source", 1000, 0);
	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

	detector->detect(image, keypoints);
	cv::drawKeypoints(gray, keypoints, blob, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::imshow("Blob", blob);
	cv::moveWindow("Blob", 1000, 500);

	if (keypoints.size() == 1)
		return cv::Point2d(keypoints[0].pt.x, keypoints[0].pt.y);

	return cv::Point2d(-1, -1);
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * initRobot()                                                                                    *
 * ---------------------------------------------------------------------------------------------- *
 */
void initRobot(ArRobot& robot, int argc, char** argv)
{
	// Connect to the robot.
	Aria::init();
	ArArgumentParser p(&argc, argv);
	p.loadDefaultArguments();
	ArRobotConnector c(&p, &robot);

	// Try to get a connection to the robot.
	if (!c.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to P3-DX.");
		if (p.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	// See if all command line parameters can be parsed.
	if (!Aria::parseArgs() || !p.checkHelpAndWarnUnparsed())
	{
		ArLog::log(ArLog::Terse, "Could not parse command line arguments.");
		Aria::logOptions();
		Aria::exit(1);
	}

	robot.lock();
	robot.runAsync(true);
	robot.disableSonar();
	robot.enableMotors();
	robot.unlock();
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * controlRobot()                                                                                 *
 * ---------------------------------------------------------------------------------------------- *
 */
void controlRobot(ArRobot& robot, double deltaX, double deltaY)
{
	assert(deltaX >= -1 && deltaX <= 1 && deltaY >= -1 && deltaY <= 1);
	robot.lock();
	robot.setVel(ROBOT_MAX_DRIVING_SPEED * deltaY);
	robot.setRotVel(ROBOT_MAX_ROTATING_SPEED * deltaX);
	robot.unlock();
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * shutdownRobot()                                                                                *
 * ---------------------------------------------------------------------------------------------- *
 */
void shutdownRobot(ArRobot& robot)
{
	robot.lock();
	robot.disableMotors();
	robot.stopRunning();
	robot.waitForRunExit();
	robot.unlock();
}
