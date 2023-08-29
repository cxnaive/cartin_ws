#ifndef _CAMERA_CLASS_
#define _CAMERA_CLASS_

#include "camera_error.h"
#include "camera_defs.h"
#include "camera_params.h"
#include <thread>
#include <mutex> 
#include <opencv2/opencv.hpp>

namespace oculii
{
    /*! ImageData struct for storing camera images
     */
    struct ImageData
    {
        std::string timestamp; /*!< Timestamp for the frame */
        cv::Mat data; /*!< Image in cv::Mat format */
    };
    class Camera
    {
        public:
            /*! Constructor with given configurations in Camera abstract class
            * @param id is an integer for the cam ID assigned
            * @param CameraInitParams is a CameraInitParams instance which contains all the configiguation paramers
            * @return None
            */
            Camera(int id, CameraInitParams CameraInitParams);

            /*! Virtual destructor for Camera class
            * @return None
            */
            virtual ~Camera();

            /*! Start to reading thread with parameters provided in CameraInitParams
            * @return CameraErrorCode, take a reference to the CameraErrorCode structe
            */
            CameraErrorCode Open( );
            
            /*! Close virtual function
            * @return None
            */
            virtual void Close() = 0;
            

            /*! Set framerate of the camera
            * @return None
            */
            void SetFrameRate(int framerate);

            /*! Get framerate of the camera
            * @return framerate of the rocorded camera
            */
            int GetFrameRate();
            
            /*! Start recording of the camera
            * @return None
            */
            void EnableRecording();

            /*! End recording of the camera
            * @return framerate of the rocorded camera
            */
            void EndRecording();

    };


    class WebCamera : public Camera
    {
        public:
            /*! Constructor with given configurations in WebCamera class
            * @param id is an integer for the cam ID assigned
            * @param CameraCameraInitParams is a CameraCameraInitParams instance which contains all the configiguation paramers
            * @return None
            */
            WebCamera(int id, std::string cameraSource, CameraInitParams CameraInitParams);

            /*! Destructor for WebCamera class which terminate reading thread and clear all memory allocated
            * @return None
            */
            ~WebCamera();

            /*! Read Cam Data and write it to the saved path, pic name is the timestamp
            * @return the cv::Mat object
            */
            cv::Mat GetOneFrame();
    };

}


#endif
