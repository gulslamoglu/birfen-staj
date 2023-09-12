# ROS Projelerim

Bu repo, Robot Operating System (ROS) üzerinde geliştirdiğim 15 farklı projeyi içermektedir. Aşağıda bu projelerin her biri hakkında kısa bir açıklama bulabilirsiniz. Her bir projenin ayrıntılı açıklamaları ve kaynak kodları ilgili klasörlerde bulunmaktadır. Ayrıca staj süresince çalışmalarımla alakalı aldığım notları içeren linke en aşağıdaki iletişim kısmında erişebilirsiniz.

## ROS Hakkında

ROS (Robot Operating System), robotlar ve otomasyon uygulamaları için hazırlanmış bir takım kütüphanelerin bir araya geldiği açık kaynaklı bir işletim sistemidir. ROS, robot kontrolü, algılama, simülasyon ve daha birçok robotik görevi gerçekleştirmek için kullanılır. 

Adının aksine bir işletim sistemi değildir. Çok yararlı yazılımlar, simülasyon araçları(GAZEBO), görselleştirme araçları(RVIZ, RQT), ağ araçları gibi bir takım araçları içinde barındırır.

## Projeler

## 1. [Elevator Control Service Example](src/elevator_control_example/scripts)

## Asansör Kontrol İstemcisi

Bu Python betiği, bir asansör kontrol hizmetini çağırmak için kullanılır. İstemci, asansörü belirli bir yüksekliğe kaldırmak veya indirmek için kullanılabilir.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve hizmetlere bağımlıdır:
- rospy
- elevator_control_example.srv.ElevatorControl


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./elevator_control_client.py
Betik, kullanıcıdan asansörü hangi yükseklikte kaldırmak istediğini sorar.

Kullanıcının girdiği yükseklik bilgisini kullanarak elevator_control hizmetini çağırır.

Hizmet çağrısı sonucunu ve iletiyi ekrana basar.

Geçersiz bir yükseklik değeri girilirse, bir hata mesajı görüntülenir.

### Notlar
Bu betik, elevator_control adlı bir ROS hizmetini çağırarak asansörü belirli bir yüksekliğe kaldırmak veya indirmek için kullanılır.

## Asansör Kontrol Sunucusu

Bu Python betiği, asansör kontrol hizmetini sunmak için kullanılır. Sunucu, asansörün belirli bir yüksekliğe kaldırılmasının veya indirilmesinin taleplerini kabul eder ve bu taleplere yanıt verir.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve hizmetlere bağımlıdır:
- rospy
- elevator_control_example.srv.ElevatorControl
- elevator_control_example.srv.ElevatorControlResponse


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./elevator_control_server.py
Sunucu, /elevator_control adlı bir ROS hizmetini başlatır.

Hizmet çağrısı geldiğinde, asansörün yüksekliğini kontrol eder.

Asansörün 5'ten daha büyük bir yüksekliği kaldıramayacağını belirler ve uygun yanıtı gönderir.

Hizmet çağrılarına yanıt olarak bir başarı durumu ve ileti döndürür.

### Notlar
Bu betik, /elevator_control adlı bir ROS hizmetini kullanarak asansörün yüksekliğini kontrol eder ve uygun yanıtları gönderir.

### Action
Asansör örneğini bir Action hizmeti olarak yeniden yazmak için Python'da **`actionlib`** kütüphanesini kullanabiliriz. 

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from elevator_control_example.msg import ElevatorControlAction, ElevatorControlResult, ElevatorControlFeedback

class ElevatorControlServer(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('elevator_control', ElevatorControlAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        # Hedef yükseklik alınır
        target_floor = goal.floor

        # Asansörü hedef yüksekliğe taşıma simülasyonu yapılır (örneğin, zaman geçişi)
        success = self.move_elevator(target_floor)

        if success:
            self.server.set_succeeded(ElevatorControlResult(success=True, message="Asansör başarıyla kaldırıldı - indirildi."))
        else:
            self.server.set_aborted(ElevatorControlResult(success=False, message="Asansör hedef yüksekliği kaldıramadı."))

    def move_elevator(self, target_floor):
        # Bu işlev asansörün hedef yüksekliğe taşınmasını simüle eder.
        # Simülasyon başarılıysa True, başarısızsa False döndürür.
        # Gerçek asansör kontrol kodu buraya gelebilir.
        # Simülasyon olarak her zaman başarılı kabul ediyoruz.
        return True

if __name__ == "__main__":
    rospy.init_node('elevator_control_server')
    elevator_server = ElevatorControlServer()
    rospy.spin()

```

Bu örnekte, asansör kontrolünü bir Action hizmetine dönüştürdük. **`ElevatorControlAction`**, **`ElevatorControlResult`**, ve **`ElevatorControlFeedback`** mesajlarını içeren **`elevator_control_example.msg`** dosyasını kullanıyoruz. **`ElevatorControlServer`** sınıfı, Action sunucusunu başlatır ve gelen hedef yüksekliği alır. Ardından, **`move_elevator`** işlevi, asansörün hedef yüksekliğe taşınmasını simüle eder (gerçek kontrol kodu burada olacaktır). Sonuç olarak, işlem başarılıysa veya başarısızsa uygun sonuçları ayarlar.

Action hizmetleri, istemci tarafından bir hedef gönderilir ve sonuçlar alınana kadar asenkron bir şekilde çalışır. Bu, istemcinin asansörün işlemini tamamlanmasını beklemesine ve istemciye ilerleme hakkında geri bildirim sağlama yeteneği sağlar.

## Action Vs Service:

1. **Service (Hizmet)**: Servisler, istemciden bir talep alır, bu talebi işler ve sonuçları anında yanıtlar. Servisler senkron bir yapıya sahiptir, yani istemci talep gönderdiğinde işlem tamamlanana kadar bekler ve sonucu alır. Servisler, istemci ile sunucu arasında iletişimde kullanılır ve tipik olarak anlık yanıtlar gerektiren işlemler için kullanılır.
2. **Action (Eylem)**: Eylemler, uzun süreli görevlerin yönetimi ve gerçek zamanlı geri bildirim sağlama amacıyla kullanılır. İstemci bir hedef gönderir, sunucu bu hedefi işlerken geri bildirim sağlar ve sonuçları gönderir. Action, işlem süresi uzun olan görevler için kullanışlıdır ve istemcinin işlemi beklemesi gerekmeksizin diğer işlemleri yürütmesine olanak tanır.

Action, gerçek zamanlı denetim ve geri bildirim gerektiren karmaşık işlemlerde kullanılırken, servisler daha basit, anlık istek-yanıt gerektiren görevler için uygundur.

## 2. [Turtle TF Broadcaster](src/learning_tf/nodes)

# Turtle TF Broadcaster

Bu Python betiği, ROS (Robot Operating System) kullanarak bir Turtlebot'un pozisyonunu ve yönelimini TF (Dönüşüm Çerçevesi) mesajlarına dönüştürmek ve bu dönüşüm bilgilerini yayınlamak için kullanılır. Bu dönüşüm bilgileri, Turtlebot'un pozisyonunu ve yönelimini "world" çerçevesi içinde ifade eder.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve mesajlara bağımlıdır:
- rospy
- tf
- turtlesim.msg.Pose


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./turtle_tf_broadcaster.py
Betik başladığında, /turtle1/pose konusundan gelen pozisyon verilerini dinler.

Her yeni pozisyon verisi geldiğinde, bu verileri TF mesajlarına dönüştürür ve "world" çerçevesi içinde Turtlebot'un pozisyonunu ve yönelimini ifade eder.

Dönüşüm bilgilerini /tf konusuna yayınlar.

### Parametreler
turtlename: Turtlebot'un adını belirler. Bu ad, parametre olarak alınabilir veya betik içinde doğrudan ayarlanabilir. 

### Notlar
Betik, /turtle1/pose konusundan gelen pozisyon verilerini dinler ve bu verileri TF dönüşüm bilgilerine çevirir.
TF dönüşümü, Turtlebot'un (x, y) pozisyonunu ve yönelimini ("world" çerçevesi içinde) ifade eder.

## 3. [Draw circle](src/my_ros_examples/scripts)

# Turtlebot ile Daire Çizme

Bu Python betiği, ROS (Robot Operating System) kullanarak bir Turtlebot'un yardımıyla bir daire çizmeye yönelik bir örnek içerir. Betik, Turtlebot'a `Twist` mesajları göndererek ileri yönde hareket etmesini ve dönmeyi sağlar, bu nedenle bir daire çizilir.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine bağımlıdır:
- rospy
- geometry_msgs.msg.Twist

### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./draw_circle.py
Betik başladığında, Turtlebot ileri yönde hareket edip dönmeye başlayacak ve böylece bir daire çizilecektir.

Daire çiziminin sürekli devam etmesi için betik döngü içinde döner ve Twist mesajları yayınlar.

### Notlar
Bu betik, Turtlebot'a dönmeyi ve ileri yönde hareket etmeyi sağlamak için Twist mesajları kullanır.
Dairenin çizim hızını ayarlamak için rate değişkenini düzenleyebilirsiniz.

## 4. [Cmd vel Publisher](src/my_ros_examples/scripts)

# cmd_vel Yayıncısı

Bu Python betiği, ROS (Robot Operating System) kullanarak bir robotun hareket komutlarını yayınlamanıza yardımcı olur. Betik, `/cmd_vel` konusuna `Twist` mesajları gönderir, böylece robot belirli bir hızda ileri yönde hareket eder.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve mesajlara bağımlıdır:
- rospy
- geometry_msgs.msg.Twist


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./cmd_vel_publisher.py
Betik başladığında, /cmd_vel konusuna Twist mesajları göndererek robotu ileri yönde 0.2 m/s hızında hareket ettirir.

Betik döngü içinde döner ve komutları belirli bir hızda yayınlamaya devam eder.

### Parametreler
Betik içinde aşağıdaki parametreleri düzenleyebilirsiniz:

cmd.linear.x: Robotun ileri yönde hareket etme hızını ayarlar. Örneğin, cmd.linear.x = 0.2 ile robot 0.2 m/s hızında ileri yönde hareket eder.
cmd.angular.z: Robotun dönme hızını ayarlar. Örneğin, cmd.angular.z = 0.0 ile robot dönmez.
### Notlar
Bu betik, /cmd_vel konusuna Twist mesajları göndererek robotun hareketini kontrol eder.

## 5. [Turtlebot Pose Publisher](src/my_ros_examples/scripts)

# Turtlebot Pozisyon Abonesi

Bu Python betiği, ROS (Robot Operating System) kullanarak bir Turtlebot'un pozisyonunu izlemek ve bu pozisyon verilerini almak için kullanılır. Betik, `/turtle1/pose` konusundan gelen `Pose` mesajlarını dinler ve bu mesajları ekrana basar.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve mesajlara bağımlıdır:
- rospy
- turtlesim.msg.Pose

### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./turtle_pose_subscriber.py
Betik başladığında, /turtle1/pose konusundan gelen Pose mesajlarını dinlemeye başlar.

Her mesajı ekrana basar, bu nedenle Turtlebot'un anlık pozisyon verilerini görebilirsiniz.

### Parametreler
Betik içinde yorum satırı olarak verilen kod parçası, pose_callback fonksiyonunu düzenler ve Pose mesajının içindeki x ve y değerlerini bir pozisyon koordinatı olarak ekrana basar. Bu değişiklik ile x ve y değerlerini daha anlamlı bir şekilde görebilirsiniz.

### Notlar
Bu betik, /turtle1/pose konusundan gelen Pose mesajlarını dinleyerek Turtlebot'un anlık pozisyon verilerini görüntüler.

## 6. [My first Publisher](src/my_ros_examples/scripts)

Basit bir publisher örneğidir.

## 7. [Turtle Controller](src/my_ros_examples/scripts)

# Turtlebot Hareket ve Çizim Kontrolü

Bu Python betiği, ROS (Robot Operating System) kullanarak bir Turtlebot'u hem hareket ettirmek hem de çizim yapmak için kullanılır. Betik, `/turtle1/pose` konusundan gelen pozisyon verilerini izler ve buna göre Turtlebot'u belirli bir koordinat aralığında hareket ettirir. Ayrıca, çizgi rengini değiştirmek için `/turtle1/set_pen` hizmetini çağırır.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve mesajlara bağımlıdır:
- rospy
- turtlesim.msg.Pose
- geometry_msgs.msg.Twist
- turtlesim.srv.SetPen


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./turtle_controller.py
Betik başladığında, /turtle1/pose konusundan gelen pozisyon verilerini dinler ve Turtlebot'u belirli bir koordinat aralığında hareket ettirir.

Turtlebot'un hareketini kontrol etmek için Twist mesajları gönderir ve /turtle1/cmd_vel konusuna yayınlar.

Turtlebot'un çizdiği çizginin rengini değiştirmek için /turtle1/set_pen hizmetini çağırır.

### Notlar
Betik, Turtlebot'un pozisyonunu izler ve belirli bir koordinat aralığında ise ileri yönde hareket ettirir, aksi takdirde döndürür.
Çizilen çizginin rengi, call_set_pen_service fonksiyonu aracılığıyla değiştirilir.
Parametreler
pose.x, pose.y: Turtlebot'un X ve Y koordinatları, belirli bir koordinat aralığını belirlemek için kullanılır.
cmd.linear.x, cmd.angular.z: Turtlebot'un ileri yönde ve dönme hızları, hareket komutlarını oluşturmak için kullanılır.
call_set_pen_service: Çizilen çizginin rengini ve kalınlığını ayarlamak için kullanılan hizmeti çağırır.

## 8. [Concatenate Strings Service Examples](src/my_string_concat_service/src)

## String Birleştirme İstemci
Bu Python betiği, ROS (Robot Operating System) kullanarak bir hizmeti çağırarak iki metin dizesini birleştirir. İşte nasıl kullanılacağına dair ayrıntılar:

### Bağımlılıklar
Bu betik, aşağıdaki ROS paketlerine ve hizmet mesajlarına bağımlıdır:
rospy
my_string_concat_service.srv.ConcatenateStrings


### Kullanım
Betiği çalıştırın:
./concatenate_strings_client.py
Betik, belirtilen iki metin dizesini birleştirmek için ilgili hizmeti çağırır.

Birleştirilmiş metin dizesi sonucunu ekranda görüntüler.

### Örnek Kullanım
Örneğin, "Hello, " ve "world!" metinlerini birleştirmek için aşağıdaki gibi kullanabilirsiniz:

./concatenate_strings_client.py
Sonuç, "Hello, world!" olacaktır.

### Notlar
Bu betik, hizmeti çağırmak için rospy.ServiceProxy kullanır.
Eğer hizmet çağrısı başarısız olursa, bir hata mesajı görüntülenir.

## String Birleştirme Sunucusu
Bu Python betiği, ROS (Robot Operating System) kullanarak iki metin dizesini birleştiren bir hizmet sunucusu oluşturur. İstemcilerden gelen taleplere yanıt verir. İşte nasıl kullanılacağına dair ayrıntılar:

### Bağımlılıklar
Bu betik, aşağıdaki ROS paketlerine ve hizmet mesajlarına bağımlıdır:

rospy
my_string_concat_service.srv.ConcatenateStrings
my_string_concat_service.srv.ConcatenateStringsResponse


### Kullanım
Betiği çalıştırın:
./concatenate_strings_server.py
Betik, "concatenate_strings" adında bir hizmet sunar ve istemcilerden gelen taleplere yanıt verir.

İstemcilerden gelen talepleri birleştirir ve yanıt olarak birleştirilmiş metin dizesini gönderir.

### Notlar
Bu betik, rospy.Service kullanarak hizmet sunar ve talepleri işler.
Hizmet sunucusu, iki metin dizesini birleştirir ve sonucu geri döndürür.
Sunucu, "Ready to concatenate strings." mesajıyla başlayarak hazır durumda bekler.

## 9. [Odom Publisher](src/my_turtlebot3_pkg/scripts)

# Odometri Verilerini Abone Etme

Bu Python betiği, ROS (Robot Operating System) kullanarak bir robotun odometri verilerini almanıza yardımcı olur. Odometri verileri, robotun konumunu ve hareketini izlemek için kullanışlıdır.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine bağımlıdır:
- rospy
- nav_msgs.msg


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./odom_verisi_al.py
Program başladığında, odom konusunu dinleyerek robotun odometri verilerini alır ve bunları günlüğe (log) yazar.

Odometri verileri, odom_callback fonksiyonu aracılığıyla alınır ve rospy.loginfo kullanılarak günlüğe (log) yazılır.

### Notlar
Bu betik, odom konusuna abone olarak odometri verilerini alır. Odometri verileri, robotun konumunu ve hareketini izlemek için kullanışlıdır.

## 10. [Robot Konumunu Alma (yeni topic örneği)](src/new_topic_example)

# Robot Konumunu Alma

Bu Python betiği, ROS (Robot Operating System) kullanarak bir robotun konumunu almanıza yardımcı olur. Betik, TF (Transform) konseptini kullanarak bir kaynaktan hedef bir çerçeve içindeki robotun konumunu alır.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine bağımlıdır:
- rospy
- tf


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./robot_konumunu_al.py
Program başladığında, TF konseptini kullanarak robotun konumunu alır ve bunu düzenli aralıklarla günlüğe (log) yazar.

Betik, TF'den robotun konumunu almak için doğru frame'leri ve hedefi ayarlamanızı gerektirir. Aşağıdaki değişkenleri düzenleyebilirsiniz:

target_frame: Hedef robotun çerçeve adı.
source_frame: Dünya koordinat sistemi çerçevesi.
Betik, TF'den veri alırken olası hataları ele alır ve günlüğe (log) yazar.

### Parametreler
Betik içinde aşağıdaki değişkenleri düzenleyebilirsiniz:

rate: Döngü hızı, TF'den konum verilerini ne sıklıkta alacağınızı belirler.

### Notlar
Bu betik, TF konseptini kullanarak robotun konumunu alır. Robotun doğru frame'leri ve TF konfigürasyonu ayarlaması gereklidir.
Betik, rospy.loginfo ve rospy.logwarn kullanarak günlüğe (log) mesajlar yazar.


## 11. [Multi Turtle Controller](src/turtlesim_example)

Bu Python betiği, ROS (Robot Operating System) kullanarak birden fazla Turtlebot'u kontrol etmek için kullanılır. Bu betik, her bir Turtlebot'un hızını ayarlamak için kullanılır.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine bağımlıdır:
- rospy
- geometry_msgs


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./multi_turtle_control.py
İlk Turtlebot'un ileri veya geri yönde hareket etmesini sağlamak için move_turtle fonksiyonunu kullanabilirsiniz. Örneğin:
move_turtle("turtle1", 1.0, 0.0)  # İlk Turtlebot ileri yönde hareket eder.
move_turtle("turtle2", -1.0, 0.0)  # İkinci Turtlebot geri yönde hareket eder.
Betik, her Turtlebot için ayrı bir kontrol düğümü başlatır ve belirtilen hızlara göre hareket eder.

### Parametreler
Betiği özelleştirmek için aşağıdaki parametreleri kullanabilirsiniz:

move_turtle fonksiyonu içinde Turtlebot adını, lineer hızı ve açısal hızı belirleyebilirsiniz.

## 12. [Draw square, circle, triangle](src/turtlesim_example)

# Turtle Shape Drawer

Bu Python betiği, ROS (Robot Operating System) kullanarak bir Turtlebot ile farklı şekiller çizmenizi sağlar. Kare, yuvarlak ve üçgen gibi temel şekilleri çizebilirsiniz.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve Python modüllerine bağımlıdır:
- rospy
- geometry_msgs.msg
- turtlesim.msg
- math modülü


### Kullanım

1. Betiği çalıştırın:
   ```bash
   ./shape_drawer.py
Program çalıştığında, aşağıdaki seçenekleri görürsünüz:

-Kare
-Yuvarlak
-Üçgen
-Çıkış
Çizmek istediğiniz şekli seçin, ardından Turtlebot o şekli çizecektir. Çizim tamamlandığında veya çıkış seçeneğini seçtiğinizde program sona erer.

### Parametreler
Betiği özelleştirmek veya farklı şekiller çizmek için kod içindeki şu yöntemleri kullanabilirsiniz:

draw_square: Kare çizer.
draw_circle: Yuvarlak çizer.
draw_triangle: Üçgen çizer.
Çizim hızını ve şekillerin boyutlarını ayarlamak için bu yöntemleri düzenleyebilirsiniz.

### Notlar
Çizimler, /turtle1/cmd_vel konusuna hareket komutları göndererek gerçekleştirilir.
Çizimlerin düzgün çalışabilmesi için Turtlebot'un turtlesim simülasyonu etkin olmalıdır.

## 13. [Map Navigation with move_base](src/topic03_map_navigation/src)

# Map Navigation with move_base

Bu Python betiği, ROS (Robot Operating System) ortamında bir mobil robotun belirli bir hedefe gitmesini sağlar. "move_base" adlı ROS paketi ve hareket tabanlı bir eylem (action) sunucusu kullanılarak, robotun bir hedef konuma gitmesi hedeflenir.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve mesajlara bağımlıdır:

rospy
actionlib
move_base_msgs.msg.MoveBaseAction
actionlib_msgs.msg.GoalStatus
geometry_msgs.msg.Point


### Kullanım

1. Betiği çalıştırın:
   ```bash
      ./map_navigation.py
Betik, "move_base" adlı hareket tabanlı eylem sunucusunu kullanarak hedefe gitmek için hareket eder.
Hedef konumu (xGoal ve yGoal) belirtilmelidir. Bu örnekte, hedef konum "-2.02880191803, 4.02200937271" olarak belirtilmiştir. Kendi hedef konumunuzu belirlemek için betiği düzenleyebilirsiniz.
Robot, hedef konuma gitmek için gerekli hesaplamaları yapar ve "move_base" eylem sunucusuna gönderir.
Robot, hedefe ulaştığında başarılı bir şekilde tamamlanmış bir işlem döndürecektir.

### Notlar
Bu betik, ROS ortamında harita tabanlı navigasyon için kullanılabilir.
Hedef konumu ve diğer parametreleri betikte düzenleyebilirsiniz.
Robotun hareketini izlemek ve hedefe ulaşıp ulaşmadığını kontrol etmek için ROS komutları kullanabilirsiniz.

## 14. [Turtlesim Odometri Bilgisi Okuma](src/topic01_quaternion/src)

# Turtlesim Odometri Bilgisi Okuma

Bu Python betiği, ROS (Robot Operating System) ortamında bir Turtlebot robotunun odometri bilgilerini okumak için kullanılır. Odometri bilgileri, robotun konumunu ve hızını içerir.

### Bağımlılıklar

Bu betik, aşağıdaki ROS paketlerine ve mesajlara bağımlıdır:

rospy
geometry_msgs.msg.Twist
nav_msgs.msg.Odometry
math
time
std_srvs.srv.Empty
tf


### Kullanım

1.Betiği çalıştırın:
    ```bash
      ./turtlesim_odometry_reader.py
Betik, "/odom" adlı konudan odometri verilerini dinlemeye başlar.

Odometri verileri her geldiğinde, odomPoseCallback fonksiyonu çağrılır ve bu veriler ekrana basılır.

Konum (x, y), hız (vx, vz) ve oryantasyon (qx, qy, qz, qw) bilgileri yazdırılır. Ayrıca bu bilgilerden hesaplanan roll, pitch ve yaw açıları da yazdırılır.

### Notlar
Bu betik, Turtlebot veya benzeri bir mobil robotun odometri bilgilerini izlemek için kullanılabilir.
Odometri verilerini işlemek veya başka işlemlerle entegre etmek için bu betiği kullanabilirsiniz.

## İletişim

Projelerle ilgili herhangi bir sorunuz veya öneriniz varsa, lütfen benimle iletişime geçmekten çekinmeyin:

**Adınız Soyadınız**
- E-posta: [gulislamogluu@hotmail.com](mailto:gulislamogluu@hotmail.com)
- GitHub: [github.com/gulslamoglu](https://github.com/gulslamoglu)
- Notlarım: [notion.com/gyuli](https://successful-helicopter-0c1.notion.site/56f9a251c4c74a8c9e70aaa24756c502?v=91a4cdc4e48349728553ea7c069c9fff&pvs=4)
