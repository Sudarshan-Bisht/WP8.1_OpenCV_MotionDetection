// main.cpp

// Copyright (c) Microsoft Open Technologies, Inc.
// All rights reserved.
//
// (3 - clause BSD License)
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that
// the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
// following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
// following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
// promote products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "pch.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/cap_winrt.hpp>

using namespace cv;
using namespace Windows::ApplicationModel::Chat;
using namespace Windows::UI::Popups;
using namespace Windows::UI::Core;
using namespace Windows::ApplicationModel::Core;
using namespace Concurrency;
using namespace concurrency;
using namespace Platform;
using namespace Windows::Media::Capture;
using namespace Windows::Media::Editing;
using namespace Windows::Media::Effects;
using namespace Windows::Media::MediaProperties;
using namespace Windows::Storage;
using namespace Windows::Foundation;
using namespace Windows::Media::Devices;
using namespace Windows::Foundation::Collections;
using namespace Platform;
using namespace Windows::Phone::Devices::Notification;


namespace MotionDetection {

	bool DetectMotion(Mat &frame, Mat &previousFrame);
	void CommandInvokedHandler(Windows::UI::Popups::IUICommand^ command);
	Mat previousFrame;
	bool res = false;
	bool msgOK = true;

	void cvMain()
	{
		
		MediaCapture^ m_mediaCapture = ref new MediaCapture();
		MediaCaptureInitializationSettings^ setting = ref new MediaCaptureInitializationSettings();
		setting->StreamingCaptureMode = StreamingCaptureMode::Video;
		m_mediaCapture->InitializeAsync(setting);
		ImageEncodingProperties^ imageProfile = ImageEncodingProperties::CreateJpeg();

		//Initializing frame counter used by face detection logic
		long frameCounter = 0;
		long frameCounterAtCapture = 0;
		bool photCapture = false;


		// open the default camera
		VideoCapture cam;
		cam.open(0);

		Mat frame;

		VibrationDevice^ testVibrationDevice = VibrationDevice::GetDefault();

		// process frames
		while (1)
		{
			// get a new frame from camera - this is non-blocking per spec
			cam >> frame;

			// don't reprocess the same frame again
			// if commented then flashing may occur
			if (!cam.grab()) continue;

			frameCounter++;

			if (frameCounter == 1)
			{
				// Just leave first frame, becuase we dont have previous frame to compare.
			}
			else
			{
				if (frame.rows != 0 && frame.cols != 0 && previousFrame.rows != 0 && previousFrame.cols != 0)
				{
					res = DetectMotion(frame, previousFrame);

					if (res == true)
					{						
						try
						{
							if (msgOK)
							{
								CoreDispatcher^ dispatcher = CoreApplication::MainView->CoreWindow->Dispatcher;
								dispatcher->RunAsync(CoreDispatcherPriority::Normal,
									ref new Windows::UI::Core::DispatchedHandler([=]() -> void
								{
									MessageDialog^ msg = ref new MessageDialog("Motion Detected !!!", "Alert !!!");
									UICommand^ continueCommand = ref new UICommand("OK", ref new UICommandInvokedHandler(CommandInvokedHandler));
									msg->Commands->Append(continueCommand);
									msg->ShowAsync();

									Windows::Foundation::TimeSpan span;
									span.Duration = 10000000L;   // convert 1 sec to 100ns ticks
									testVibrationDevice->Vibrate(span);

								}));
							}

							msgOK = false;
						}
						catch (...)
						{

						}

						frameCounterAtCapture = frameCounter;
					}
				}
			}

			if (frame.rows != 0 && frame.cols != 0)
			{
				previousFrame = frame;
			}

			// important step to get XAML image component updated
			winrt_imshow();
		}
	}

	void CommandInvokedHandler(Windows::UI::Popups::IUICommand^ command)
	{
		if (command->Label == "OK")
		{
			msgOK = true;
		}
	}

	bool DetectMotion(Mat &frame, Mat &previousFrame)
	{
		bool result = false;

		Mat currentFrameGreyscale;
		Mat previousFrameGreyscale;
		Mat currentGaussianBlur;
		Mat previousGaussianBlur;
		std::vector<uchar> diff;
		std::vector<uchar> outputArray;

		// Convert frames to greyscale
		cvtColor(frame, currentFrameGreyscale, CV_RGB2GRAY);
		cvtColor(previousFrame, previousFrameGreyscale, CV_RGB2GRAY);

		// Apply Gaussian blur to smooth out frames
		GaussianBlur(currentFrameGreyscale, currentGaussianBlur, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
		GaussianBlur(previousFrameGreyscale, previousGaussianBlur, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);


		// Convert frames to array
		std::vector<uchar> array1;
		if (currentGaussianBlur.isContinuous()) 
		{
			array1.assign(currentGaussianBlur.datastart, currentGaussianBlur.dataend);
		}
		else 
		{
			for (int i = 0; i < currentGaussianBlur.rows; ++i) 
			{
				array1.insert(array1.end(), currentGaussianBlur.ptr<uchar>(i), currentGaussianBlur.ptr<uchar>(i)+currentGaussianBlur.cols);
			}
		}

		std::vector<uchar> array2;
		if (previousGaussianBlur.isContinuous()) 
		{
			array2.assign(previousGaussianBlur.datastart, previousGaussianBlur.dataend);
		}
		else 
		{
			for (int i = 0; i < previousGaussianBlur.rows; ++i) 
			{
				array2.insert(array2.end(), previousGaussianBlur.ptr<uchar>(i), previousGaussianBlur.ptr<uchar>(i)+previousGaussianBlur.cols);
			}
		}

		// Find the absolute diff b/w current and porevious data
		absdiff(array1, array2, diff);

		// Filter out non-significant information, only take 25->255 into consideration
		double thresh = cv::threshold(diff, outputArray, 25, 255, cv::THRESH_BINARY);

		// Check array if it contains any data of value 25 or more, if yes, then there is a motion
		for (unsigned int i = 0; i < outputArray.size(); i++)
		{
			if (outputArray[i] >= 25)
			{
				OutputDebugStringA("\n Motion Detected !!!");
				result = true;
				break;
			}
		}

		return result;
	}
}
