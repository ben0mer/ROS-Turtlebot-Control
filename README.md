### TurtleBotRace.cpp
Bu projede, iki TurtleBot3 robotu bir yarış pistine yerleştirilir. Robotlar, engelleri algılamak ve onlardan kaçınmak için lazer tarama sensörlerini kullanır. Hedef, belirli bir koordinatta (örn. x = 5.0, y = 0.0) bulunur. Robotlar, engelleri algıladıklarında onlardan kaçınmak için sola dönerler. Engelden uzakta olduklarında ise hedefe doğru hareket ederler.

### TurtleBotControl.cpp
Bu proje, iki farklı modda çalışan bir TurtleBot3 robotunu içermektedir

####  Klavye Kontrol Modu
Bu modda, klavyeden alınan kullanıcı girişi ile robotu hareket ettirebilirsiniz. Aşağıdaki tuşları kullanarak robotu kontrol edebilirsiniz:
- 'W' tuşu: İleri hareket
- 'S' tuşu: Geri hareket
- 'A' tuşu: Sola dönme
- 'D' tuşu: Sağa dönme
- 'Q' tuşu: Moddan çıkış

#### Engelden Kaçarak Hedefe Ulaşma Modu
Bu modda, robotun önünde 70 cm mesafede bir engel algılayabilen bir sensör olduğu varsayılmaktadır. Robot, hedefe ulaşmak için engellerden kaçınarak en kısa rota üzerinden hareket eder.
