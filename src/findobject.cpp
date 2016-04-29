#include "findobject.h"

FindObject::FindObject()
{
}

bool FindObject::isInHull(pcl::PointXYZ point_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull)
{
    double xmin, xmax, ymin, ymax;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud_hull, minPt, maxPt);
    xmin = minPt.x;
    ymin = minPt.y;
    xmax = maxPt.x;
    ymax = maxPt.y;

    double midx = ( xmax + xmin  ) / 2;
    double midy = ( ymax + ymin ) / 2;

    double distemp_1 = 10.0;
    double distemp_2 = 10.0;
    int p_1, p_2;
    double p1x, p1y;
    double p2x = 0.0;
    double p2y = 0.0;
    for (size_t p = 0; p < cloud_hull->points.size(); p++)
        {
            double pxt = cloud_hull->points[p].x;
            double pyt = cloud_hull->points[p].y;

            double dis = pow(pxt - point_in.x, 2 ) + pow (pyt - point_in.y, 2);
            if (dis < distemp_1)
                {
                    if (dis < distemp_2)
                        {
                            distemp_1 = distemp_2;
                            distemp_2 = dis;
                            if (p2x != 0 && p2y !=0)
                                {
                                    p1x = p2x;
                                    p1y = p2y;
                                }
                            p2x = pxt;
                            p2y = pyt;
                        }
                    else
                        {
                            distemp_1 = dis;
                            p1x = pxt;
                            p1y = pyt;
                        }
                }
        }

    //construct a line
    if (p1x != p2x)
        {
            double u = point_in.y - (p1y - p2y) / (p1x - p2x) * point_in.x - (p1x * p2y - p2x * p1y) / (p1x - p2x);
            double umid = midy - (p1y - p2y) / (p1x - p2x) * midx - (p1x * p2y - p2x * p1y) / (p1x - p2x);
            if (u * umid > 0 )
                {
                    return true;
                }
            else
                {
                    return false;
                }
        }
    else
        {
            if ( (midx - p1x) * (point_in.x - p1x) > 0)
                {
                    return true;
                }
            else
                {
                    return false;
                }
        }
}
