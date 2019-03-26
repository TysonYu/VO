//
// create by Tiezheng YU on 3/10/2019
//

#include <data_loader.h>

namespace VO
{
//--------- Calibration -----------------------------------------------------------------
Calibration::Calibration()
{
    GetCalibration("/home/icey/Desktop/project/camera_localization/data/mav0/cam0/sensor.yaml", "/home/icey/Desktop/project/camera_localization/data/mav0/cam1/sensor.yaml");
}

void Calibration::GetCalibration(const string &camera_calibration0,const string &camera_calibration1)
{
    //---------cam0-------------------------------------------------------------------
    bool FSflag0 = false;
    cv::FileStorage fCalibration0;
    FSflag0 = fCalibration0.open(camera_calibration0, cv::FileStorage::READ);
    if (FSflag0 == false) cout << "Cannot open the file" << endl;

    camera_width = fCalibration0["resolution"][0];
    camera_height = fCalibration0["resolution"][1];
    camera0_fu = fCalibration0["intrinsics"][0];
    camera0_fv = fCalibration0["intrinsics"][1];
    camera0_cu = fCalibration0["intrinsics"][2];
    camera0_cv = fCalibration0["intrinsics"][3];
    distortion_coefficients0[0] = fCalibration0["distortion_coefficients"][0];
    distortion_coefficients0[1] = fCalibration0["distortion_coefficients"][1];
    distortion_coefficients0[2] = fCalibration0["distortion_coefficients"][2];
    distortion_coefficients0[3] = fCalibration0["distortion_coefficients"][3];
    rt_cam02body = (cv::Mat_<double>(4, 4) << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0);
    //---------cam1-------------------------------------------------------------------
    bool FSflag1 = false;
    cv::FileStorage fCalibration1;
    FSflag1 = fCalibration1.open(camera_calibration1, cv::FileStorage::READ);
    if (FSflag1 == false) cout << "Cannot open the file" << endl;

    camera1_fu = fCalibration1["intrinsics"][0];
    camera1_fv = fCalibration1["intrinsics"][1];
    camera1_cu = fCalibration1["intrinsics"][2];
    camera1_cv = fCalibration1["intrinsics"][3];
    distortion_coefficients1[0] = fCalibration1["distortion_coefficients"][0];
    distortion_coefficients1[1] = fCalibration1["distortion_coefficients"][1];
    distortion_coefficients1[2] = fCalibration1["distortion_coefficients"][2];
    distortion_coefficients1[3] = fCalibration1["distortion_coefficients"][3];
    rt_cam12body = (cv::Mat_<double>(4, 4) << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
         0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
        -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
         0.0, 0.0, 0.0, 1.0);
}

//--------- Name --------------------------------------------------------------------------------------------------
Name::Name()
{
    GetTimetamp();
}

void Name::GetTimetamp()
{
    ifstream fin("/home/icey/Desktop/project/camera_localization/data/mav0/cam0/data.csv"); //打开文件流操作
    string line; 
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			timestamps.push_back(field); //将刚刚读取的字符串添加到向量groundtruth中
            break;
		}
	}
}
//---------GroundTruth-----------------------------------------------------------------------------------------------

void GroundTruth::LoadGroundTruth()
{
    ifstream fin ("/home/icey/Desktop/project/camera_localization/data/mav0/state_groundtruth_estimate0/data.csv");
    string short_timestamps;
    short_timestamps = camera_time_stamp;
    short_timestamps.pop_back();
    short_timestamps.pop_back();
    short_timestamps.pop_back();
    short_timestamps.pop_back();
    cout << "short_timestamps = " << short_timestamps << endl;
    string line;
    int count = 0;
    int flag = 0;
    while(getline(fin,line))
    {
        istringstream sin(line);
        string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
		    groundtruth.push_back(field); //将刚刚读取的字符串添加到向量groundtruth中
            groundtruth.at(0).pop_back();
            groundtruth.at(0).pop_back();
            groundtruth.at(0).pop_back();
            groundtruth.at(0).pop_back();
            // cout << "short_groundtruth = " << groundtruth.at(0) << endl;
            if (groundtruth.at(0) == short_timestamps)
            {
                flag = 1;
                cout << "Ground Truth Successfully finded!" << endl;
                groundtruth.pop_back();
                for(int i = 0; i < 7; i++)
                {
                    getline(sin, field, ',');
                    groundtruth.push_back(field);
                }
                // std::cout << count << std::endl;
            }
            else
            {
                groundtruth.pop_back();
                break;
            }
            
		}
        if (flag == 1) break;
        count ++;
    }
    if(flag == 0)
        cout << "Can NOT Find Ground Truth POSE!" << endl;
}

//--------- load test timestamps------------------------------------------
void TestName::LoadTimestamps()
{
    ifstream fin("/home/icey/Desktop/project/camera_localization/data/V1_02_medium/mav0/cam0/data.csv"); //打开文件流操作
    string line; 
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			timestamps.push_back(field); //将刚刚读取的字符串添加到向量groundtruth中
            break;
		}
	}
}

std::string TestName::GetImageName(int test_id)
{
    string result;
    result = "/home/icey/Desktop/project/camera_localization/data/V1_02_medium/mav0/cam0/data/"+ timestamps.at(test_id) +".png";
    return result;
}

}