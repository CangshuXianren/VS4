#include "front_gps2xy.h"

GPS2XY::GPS2XY(ros::Publisher pub)
{
    this->pub = pub;
    return;
}

// void GPS2XY::transform(double l, double B, double xc, double yc, double yaw, double uwb_stamp)
// {
//     l = l * M_PI /180;
//     B = B * M_PI /180;

//     double B0 =30* M_PI /180;

//     double N =0, e =0, a =0, b =0, e2 =0, K =0;
//     a =6378137;
//     b =6356752.3142;
//     e = sqrt(1- (b / a) * (b / a));
//     e2 = sqrt((a / b) * (a / b) -1);
//     double CosB0 = cos(B0);
//     N = (a * a / b) / sqrt(1+ e2 * e2 * CosB0 * CosB0);
//     K = N * CosB0;

//     double SinB = sin(B);

//     double tantan = tan(M_PI /4+ B /2);
//     double E2 = pow((1- e * SinB) / (1+ e * SinB), e /2);
//     double xx = tantan * E2;

//     xc = K * log(xx);
//     yc = K * l;

//     custom_messages::vehicle_status msg;
//     msg.xPos = xc;
//     msg.yPos = yc;
//     msg.yaw = yaw;
//     msg.header.stamp = (ros::Time)uwb_stamp;

//     pub.publish(msg);
//     return;
// }

void transform(double jd, double wd, short DH, short DH_width, double& y, double& x, double LP)
{
    double t;     //  t=tgB
    double L;     //  中央经线的经度
    double l0;    //  经差
    double jd_hd, wd_hd;  //  将jd、wd转换成以弧度为单位
    double et2;    //  et2 = (e' ** 2) * (cosB ** 2)
    double N;     //  N = C / sqrt(1 + et2)
    double X;     //  克拉索夫斯基椭球中子午弧长
    double m;     //  m = cosB * PI/180 * l0
    double tsin, tcos;   //  sinB,cosB
    double PI = 3.14159265358979;
    double b_e2 = 0.0067385254147;
    double b_c = 6399698.90178271;
    jd_hd = jd / 3600.0 * PI / 180.0;    // 将以秒为单位的经度转换成弧度
    wd_hd = wd / 3600.0 * PI / 180.0;    // 将以秒为单位的纬度转换成弧度

    // 如果不设中央经线（缺省参数: -1000），则计算中央经线，
    // 否则，使用传入的中央经线，不再使用带号和带宽参数
    //L = (DH - 0.5) * DH_width ;      // 计算中央经线的经度
    if (LP == -1000)
    {
        L = (DH - 0.5) * DH_width;      // 计算中央经线的经度
    }
    else
    {
        L = LP;
    }

    l0 = jd / 3600.0 - L;       // 计算经差
    tsin = sin(wd_hd);        // 计算sinB
    tcos = cos(wd_hd);        // 计算cosB
    // 计算克拉索夫斯基椭球中子午弧长X
    X = 111134.8611 / 3600.0 * wd - (32005.7799 * tsin + 133.9238 * pow(tsin, 3)
          + 0.6976 * pow(tsin, 5) + 0.0039 * pow(tsin, 7)) * tcos;
    et2 = b_e2 * pow(tcos, 2);      //  et2 = (e' ** 2) * (cosB ** 2)
    N = b_c / sqrt(1 + et2);      //  N = C / sqrt(1 + et2)
    t = tan(wd_hd);         //  t=tgB
    m = PI / 180 * l0 * tcos;       //  m = cosB * PI/180 * l0
    x = X + N * t * (0.5 * pow(m, 2)
              + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0
              + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
    y = N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0
                    + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2
                       - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);

}

void GPS2XY::transformCallback(const nlink_parser::LinktrackNodeframe0::ConstPtr& msg)
{

    //从string中提取经纬度和方位角数据
    std::string ll, bb, ori_yaw, ori_uwb_stamp; 
    int parser_flag = 0;    
    for(int i = 0; i < msg->nodes[0].data.size(); i++)
    {
        if(msg->nodes[0].data[i] == ',' && parser_flag == 0)
        {
            i++;
            while(msg->nodes[0].data[i] != ',')
            {
                bb+=(char)msg->nodes[0].data[i];
                i++;
            }
            i--;
            parser_flag = 1;
            continue;
        }
        if(msg->nodes[0].data[i] == ',' && parser_flag == 1)
        {
            i++;
            while(msg->nodes[0].data[i] != ',')
            {
                ll+=(char)msg->nodes[0].data[i];
                i++;
            }
            i--;
            parser_flag = 2;
            continue;
        }
        if(msg->nodes[0].data[i] == ',' && parser_flag == 2)
        {
            i++;
            while(msg->nodes[0].data[i] != ',')
            {
                ori_yaw+=(char)msg->nodes[0].data[i];
                i++;
            }
            i--;
            parser_flag = 3;
            continue;
        }
        if(msg->nodes[0].data[i] == ',' && parser_flag == 3)
        {
            i++;
            while(msg->nodes[0].data[i] != ',')
            {
                ori_uwb_stamp+=(char)msg->nodes[0].data[i];
                i++;
            }
        }
    }
    double l = atof(ll.c_str());
    double b = atof(bb.c_str());
    double yaw = atof(ori_yaw.c_str());
    double uwb_stamp = atof(ori_uwb_stamp.c_str());

    //转换为double类型并进行发布
    double xc,yc;
    transform(l*3600, b*3600, 39, 3, yc, xc, -1000);
    custom_messages::vehicle_status msg2;
    msg2.xPos = xc;
    msg2.yPos = yc;
    msg2.yaw = yaw;

    pub.publish(msg2);
    return;
}

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "front_GPS2XY");  

  ros::NodeHandle n;

  ros::Publisher pubMessage = n.advertise<custom_messages::vehicle_status>("front_GPS2XY", 10); 

  GPS2XY* front_GPS2XY = new GPS2XY(pubMessage);

  ros::Subscriber sub = n.subscribe("/nlink_linktrack_nodeframe0", 10, &GPS2XY::transformCallback, front_GPS2XY);

  ros::spin();  

  return 0;  
}  