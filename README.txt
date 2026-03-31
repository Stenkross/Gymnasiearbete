Vår titel på projektet är Tigerkross

Här är koden för projektet. Groundstation är all kod för markstationen och kodpasatellit är all kod för CanSaten. Resten är kod för individuella komponenter eller funktioner. 

Groundstation Tar emot signaler från CanSaten via en radiomodul och blinkar en lampa när en signal har mottagits. Ljusstyrkan är beroende på signalstyrkan. Datan som mottagits sparas på datorn lokalt som är kopplad till markstationen genom Coolterm som sparar Seriell monitor data. 

Kodpasatellit mäter lufttryck och temperatur, får GPS-data och sparar den datan lokalt i ett SD-kort och skickar den till Groundstation med en radiomodul. Den styr även fallskärmen med ett servo beroende på målkoordinater och GPS-datan.