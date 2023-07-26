#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>

class TurtleBot3RacingGame
{
public:
  TurtleBot3RacingGame()
  {
    // Robot hareket kontrolü için yayıncıları tanımla
    cmd_vel_pub_1_ = nh_.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 10);
    cmd_vel_pub_2_ = nh_.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 10);

    // Laser tarama verilerini almak için aboneleri tanımla
    laser_sub_1_ = nh_.subscribe("/robot1/scan", 1, &TurtleBot3RacingGame::laserCallback1, this);
    laser_sub_2_ = nh_.subscribe("/robot2/scan", 1, &TurtleBot3RacingGame::laserCallback2, this);

    // Robot hedef konumlarını tanımla
    target_x_ = 5.0;
    target_y_ = 0.0;
  }

  // İki robotu da yarışa başlatan fonksiyon
  void startRace()
  {
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      // Robot 1 hareket kontrolü
      if (!isRobot1Finished_)
      {
        geometry_msgs::Twist msg;
        if (obstacleDetected1())
        {
          // Engel algılandı, engelden kaçınmak için sola dön
          msg.angular.z = 0.5;
        }
        else
        {
          // Engel yok, hedefe doğru hareket et
          double angle_to_target = std::atan2(target_y_ - robot1_y_, target_x_ - robot1_x_);
          msg.angular.z = 0.5 * angle_to_target;
          msg.linear.x = 0.2;
        }

        cmd_vel_pub_1_.publish(msg);
      }

      // Robot 2 hareket kontrolü
      if (!isRobot2Finished_)
      {
        geometry_msgs::Twist msg;
        if (obstacleDetected2())
        {
          // Engel algılandı, engelden kaçınmak için sola dön
          msg.angular.z = 0.5;
        }
        else
        {
          // Engel yok, hedefe doğru hareket et
          double angle_to_target = std::atan2(target_y_ - robot2_y_, target_x_ - robot2_x_);
          msg.angular.z = 0.5 * angle_to_target;
          msg.linear.x = 0.2;
        }

        cmd_vel_pub_2_.publish(msg);
      }

      // İki robot da yarışı tamamladı mı kontrol et
      if (!isRobot1Finished_ && isTargetReached(robot1_x_, robot1_y_))
      {
        ROS_INFO("Robot 1 hedefe ulaştı!");
        isRobot1Finished_ = true;
      }

      if (!isRobot2Finished_ && isTargetReached(robot2_x_, robot2_y_))
      {
        ROS_INFO("Robot 2 hedefe ulaştı!");
        isRobot2Finished_ = true;
      }

      // İki robot da yarışı tamamladıysa döngüden çık
      if (isRobot1Finished_ && isRobot2Finished_)
        break;

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_1_;
  ros::Publisher cmd_vel_pub_2_;
  ros::Subscriber laser_sub_1_;
  ros::Subscriber laser_sub_2_;
  double robot1_x_ = 0.0;
  double robot1_y_ = 0.0;
  double robot2_x_ = 0.0;
  double robot2_y_ = 0.0;
  double target_x_;
  double target_y_;
  bool isRobot1Finished_ = false;
  bool isRobot2Finished_ = false;

  // Robot 1 için lazer tarama geri araması, engel algılama için kullanılır
  void laserCallback1(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    // Lazer tarama verilerini güncelle
    // Gerçek robot üzerinde uygun sensör ve algoritmalarla yapılmalıdır.
    // Örnek olarak robotun pozisyonunu güncelle:
    robot1_x_ = 0.0;
    robot1_y_ = 0.0;
  }

  // Robot 2 için lazer tarama geri araması, engel algılama için kullanılır
  void laserCallback2(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    // Lazer tarama verilerini güncelle
    // Gerçek robot üzerinde uygun sensör ve algoritmalarla yapılmalıdır.
    // Örnek olarak robotun pozisyonunu güncelle:
    robot2_x_ = 0.0;
    robot2_y_ = 0.0;
  }

  // Robot 1 için engel algılama
  bool obstacleDetected1()
  {
    // Sensör verisine göre engel algılama kontrolü
    // Örnek olarak 0.7 m'den küçük bir mesafe algılandığını varsayalım.
    // Gerçek robot üzerinde uygun sensör ve algoritmalarla yapılmalıdır.
    return false;
  }

  // Robot 2 için engel algılama
  bool obstacleDetected2()
  {
    // Sensör verisine göre engel algılama kontrolü
    // Örnek olarak 0.7 m'den küçük bir mesafe algılandığını varsayalım.
    // Gerçek robot üzerinde uygun sensör ve algoritmalarla yapılmalıdır.
    return false;
  }

  // Hedefe ulaşılıp ulaşılmadığını kontrol eden fonksiyon
  bool isTargetReached(double robot_x, double robot_y)
  {
    // Gerçek robot üzerinde uygun algoritmalarla yapılmalıdır.
    // Örnek olarak hedefe ulaşıldığını kontrol eden fonksiyon.
    return std::abs(target_x_ - robot_x) < 0.1 && std::abs(target_y_ - robot_y) < 0.1;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_racing_game");
  TurtleBot3RacingGame racing_game;

  ROS_INFO("Yarış başlıyor!");
  racing_game.startRace();
  ROS_INFO("Yarış tamamlandı!");

  return 0;
}