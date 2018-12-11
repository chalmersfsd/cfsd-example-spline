/*
 * Copyright (C) 2018  Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "splinetoolbox.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("name")) || (0 == commandlineArguments.count("cid")) || (0 == commandlineArguments.count("width")) || (0 == commandlineArguments.count("height")) ) {
    std::cerr << argv[0] << " accesses video data using shared memory provided using the command line parameter --name=." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --width=<width> --height=<height> --name=<name for the associated shared memory> [--verbose]" << std::endl;
    std::cerr << "         --width:     width of a frame" << std::endl;
    std::cerr << "         --height:    height of a frame" << std::endl;
    std::cerr << "         --name:      name of the shared memory to use" << std::endl;
    std::cerr << "         --verbose: when set, the image contained in the shared memory is displayed" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --name=cam0 --width=640 --height=480" << std::endl;
    retCode = 1;
  } else {
    std::string const name{(commandlineArguments["name"].size() != 0) ? commandlineArguments["name"] : "/cam0"};

    uint16_t const cid{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    uint32_t const width{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
    bool const verbose{commandlineArguments.count("verbose") != 0};

    uint32_t aimPointX;
    uint32_t aimPointY;
    std::mutex aimPointMutex;

    std::vector<cv::Point> controlPoints = {
      cv::Point(160, 237), cv::Point(243, 185), cv::Point(340, 174), 
      cv::Point(436, 205), cv::Point(533, 177), cv::Point(593, 102)
    };

    auto onDirection{[&aimPointX, &aimPointY, &aimPointMutex](cluon::data::Envelope &&envelope)
      {
        std::lock_guard<std::mutex> lock(aimPointMutex);
        auto const direction = cluon::extractMessage<opendlv::logic::sensation::Direction>(std::move(envelope));
        aimPointX = static_cast<uint32_t>(direction.azimuthAngle());
        aimPointY = static_cast<uint32_t>(direction.zenithAngle());
      }};

    cluon::OD4Session od4{cid};
    od4.dataTrigger(opendlv::logic::sensation::Direction::ID(), onDirection);

    std::unique_ptr<cluon::SharedMemory> sharedMemory(new cluon::SharedMemory{name});
    if (sharedMemory && sharedMemory->valid()) {
      std::clog << argv[0] << ": Found shared memory '" << sharedMemory->name() << "' (" << sharedMemory->size() << " bytes)." << std::endl;

      while (od4.isRunning()) {
        cv::Mat img;
        sharedMemory->wait();
        sharedMemory->lock();
        {
          cv::Mat imgMem(height, width, CV_8UC4, sharedMemory->data());
          img = imgMem.clone();
        }
        sharedMemory->unlock();

        int32_t x0 = controlPoints[1].x;
        int32_t y0 = controlPoints[1].y;

        for (uint32_t i = 0; i < controlPoints.size() - 3; i++) {
          cv::Point p0 = controlPoints[i];
          cv::Point p1 = controlPoints[i+1];
          cv::Point p2 = controlPoints[i+2];
          cv::Point p3 = controlPoints[i+3];

          for (double t = 0.0; t < 1.0; t += 0.05) {
            double x = splinetoolbox::catmullrom(p0.x, p1.x, p2.x, p3.x, t);
            double y = splinetoolbox::catmullrom(p0.y, p1.y, p2.y, p3.y, t);

            int32_t x1 = static_cast<int32_t>(std::round(x));
            int32_t y1 = static_cast<int32_t>(std::round(y));
            
            cv::line(img, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255, 255, 0), 3, 1);

            x0 = x1;
            y0 = y1;
          }
        }
        
        for (uint32_t i = 0; i < controlPoints.size(); i++) {
          cv::Point p = controlPoints[i];
          if (i == 0 || i == controlPoints.size() - 1) {
            cv::circle(img, p, 10, cv::Scalar(0, 0, 255), 3, 8);
          } else {
            cv::circle(img, p, 10, cv::Scalar(255, 0, 0), 3, 8);
          }
        }
     
        {
          std::lock_guard<std::mutex> lock(aimPointMutex);
          cv::circle(img, cv::Point(aimPointX, aimPointY), 10, cv::Scalar(0, 255, 0), 3, 8);
        }

        if (verbose) {
          cv::imshow("Example, spline", img);
          cv::waitKey(1);
        }
      }

    } else {
      std::cerr << argv[0] << ": Failed to access shared memory '" << name << "'." << std::endl;
    }
  }
  return retCode;
}
