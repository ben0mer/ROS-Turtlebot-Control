#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>

class TurtleBot3Controller
{
public:
  TurtleBot3Controller()
  {
    // ROS düğümünü başlat ve yayıncı ve abone olacağımız konuları tanımla
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    laser_sub_ = nh_.subscribe("/scan", 1, &TurtleBot3Controller::laserCallback, this);
  }

  // Klavye kontrol modu için döngü
  void keyboardControlLoop()
  {
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      std::cout << "Klavye kontrol modu. W: İleri, A: Sol, D: Sağ, S: Geri, Q: Çıkış" << std::endl;
      char key;
      std::cin >> key;

      if (key == 'q' || key == 'Q')
        break;

      geometry_msgs::Twist msg;
      switch (key)
      {
        case 'w':
        case 'W':
          msg.linear.x = 0.2;
          break;
        case 's':
        case 'S':
          msg.linear.x = -0.2;
          break;
        case 'a':
        case 'A':
          msg.angular.z = 0.5;
          break;
        case 'd':
        case 'D':
          msg.angular.z = -0.5;
          break;
        default:
          msg.linear.x = 0.0;
          msg.angular.z = 0.0;
          break;
      }

      cmd_vel_pub_.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  // Koordinat kontrol modu için döngü
  void coordinateControlLoop()
  {
    double target_x, target_y;
    std::cout << "Koordinat kontrol modu. Hedef x ve y koordinatlarını girin: ";
    std::cin >> target_x >> target_y;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      if (isTargetReached(target_x, target_y))
      {
        std::cout << "Hedefe ulaşıldı!" << std::endl;
        break;
      }

      geometry_msgs::Twist msg;

      if (obstacleDetected())
      {
        // Engelden kaçınmak için robotu döndür
        msg.angular.z = 0.5;
      }
      else
      {
        // Engel yoksa hedefe doğru hareket et
        double angle_to_target = std::atan2(target_y - current_y_, target_x - current_x_);
        msg.angular.z = 0.5 * angle_to_target;
        msg.linear.x = 0.2;
      }

      cmd_vel_pub_.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber laser_sub_;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double laser_range_ahead_ = 0.0;

  // Lazer tarama geri araması, engel algılama için kullanılır
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    // Lazer tarama verilerini güncelle
    laser_range_ahead_ = msg->ranges[msg->ranges.size() / 2];
    // Örnek olarak robotun pozisyonunu güncellemek için kullanabilirsiniz.
    // Gerçek robot üzerinde bu işlemi konum verisini elde ederek yapmalısınız.
    current_x_ = 0.0;
    current_y_ = 0.0;
  }

  // Engelin algılanıp algılanmadığını kontrol eden fonksiyon
  bool obstacleDetected()
  {
    // Sensör verisine göre engel algılama kontrolü
    // laser_range_ahead_ değişkeni sensörden gelen 70 cm ilerisindeki mesafeyi tutuyor.
    // Örnek olarak 0.7 m'den küçük bir mesafe algılandığını varsayalım.
    // Gerçek robot üzerinde uygun sensör ve algoritmalarla yapılmalıdır.
    return (laser_range_ahead_ < 0.7);
  }

  // Hedefe ulaşılıp ulaşılmadığını kontrol eden fonksiyon
  bool isTargetReached(double target_x, double target_y)
  {
    // Örnek olarak hedefe ulaşıldığını kontrol eden fonksiyon.
    // Gerçek robot üzerinde uygun algoritmalarla yapılmalıdır.
    return std::abs(target_x - current_x_) < 0.1 && std::abs(target_y - current_y_) < 0.1;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_control");
  TurtleBot3Controller controller;

  std::cout << "Kontrol Modu Seçin: K: Klavye, C: Koordinat: ";
  char mode;
  std::cin >> mode;

  if (mode == 'k' || mode == 'K')
  {
    controller.keyboardControlLoop();
  }
  else if (mode == 'c' || mode == 'C')
  {
    controller.coordinateControlLoop();
  }
  else
  {
    std::cout << "Geçersiz mod seçildi. Program sonlandırılıyor." << std::endl;
  }

  return 0;
}