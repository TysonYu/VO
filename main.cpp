//
// create by Tiezheng YU on 3/14/2019
//

#include <LoadData.h>


int main(int argc, char **argv)
{
    //--------- 测试集第263个图片 ---------------训练集第2051个图片----------
    pcl::visualization::PCLVisualizer viewer("result");
    PNPSolver solver;
    // solver.KeyPoints.ShowKeyPointCloud();
    for(int i = 120; i < 1530; i = i+10)
    {
        std::cout << " test image number = " << i << std::endl;
        cv::waitKey(0);
        clock_t start = clock();
        solver.FindMatches(i);
        clock_t end = clock();
        std::cout << "=========================================================================" << std::endl;
        std::cout << "one frame use time = " << (double)(end-start)/CLOCKS_PER_SEC <<  " s"<< std::endl;
        // solver.SolvePNP();
        solver.ShowResults();
        std::cout << "starting show results ..." << std::endl;
        viewer.addPointCloud(solver.result_cloud.makeShared(),solver.TestImageName.timestamps.at(i));
        viewer.setBackgroundColor(0,0,0);
        viewer.addCoordinateSystem();
        viewer.spin();
        viewer.removeCoordinateSystem();
        viewer.removeAllPointClouds(); 
    }
    

    return 0;
}