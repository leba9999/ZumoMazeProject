#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include<stdlib.h>

#if 1
// Black and white destinations
#define BLACK 1
#define WHITE 0
// Direction
#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3
// All intersections
#define FULL_CROSS 0xf // Täys risteys
#define LEFT_T_CROSS 0xb // Vasemman puoleinen suunta puuttuu
#define RIGHT_T_CROSS 0x7 // Oikean puoleinen suunta puuttuu
#define DOWN_T_CROSS 0xd // Alaspäin oleva suunta puuttuuu
#define UP_T_CROSS 0xe // Ylöspäin oleva suunta puuttuuu
#define DOWN_LEFT_CORNER_CROSS 0x8 // Alaspäin ja vasemalle olevat suunnat puuttuu
#define DOWN_RIGHT_CORNER_CROSS 0x5 // Alaspäin ja oikeale olevat suunnat puuttuu
#define UP_LEFT_CORNER_CROSS 0xa // Ylöspäin ja vasemalle olevat suunnat puuttuu
#define UP_RIGHT_CORNER_CROSS 0x6 // Alaspäin ja oikeale olevat suunnat puuttuu
#define NO_CROSS 0x0 // Ei risteystä ollenkaan

// map sisältää kentän gridin(risteykset)
int map[15][7] = {
// Hexa luvut tulevat neljästä bitistä
// niin montaa ei tarvitsisi mutta vaikuttaa yksinkertaisemmalta
// antaa jokaiselle risteyksen tielle oma bitti eli:
// Right Left Down Up
//   1    1    1    1 = FULL_CROSS = 0xf
    
//    0    1    2    3    4    5    6
    {0x8, 0xd, 0xd, 0xf, 0xd, 0xd, 0x5},// 0
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 1
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 2
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 3
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 4
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 5
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 6
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 7
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 8
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 9
    {0xb, 0xf, 0xf, 0xf, 0xf, 0xf, 0x7},// 10
    {0xa, 0xf, 0xf, 0xf, 0xf, 0xf, 0x6},// 11
    {0x0, 0xa, 0xf, 0xf, 0xf, 0x6, 0x0},// 12
    {0x0, 0x0, 0xa, 0xf, 0x6, 0x0, 0x0},// 13
    {0x0, 0x0, 0x0, 0xf, 0x0, 0x0, 0x0},// "14"
// "14" riviä ei ole gridillä olemassa mutta se tarvitaan jotta voidaan vielä
// käyttää samaa toiminta periaatetta suoralla kuin risteyksissä
};
// robotin tiedot
struct tankInfo{
    // robotin koordinaatit
    int positionX;
    int positionY;
    // robotin suunta eli mihin suuntaan robotti osoittaa
    int direction;
} tank;
// Keijon drive_to funktio ajaa joko valkoiselle alueelle tai mustalle viivalle
void drive_to(int target)
{
    struct sensors_ dig;
    // ajetaan niin pitkään kunnes laitimmaiset sensorit näkevät mustaa tai valkoista
    // riippuu minkä asettaa targetiksi
    do {
        motor_forward(40, 5);
        reflectance_digital(&dig);
    } while(! (dig.R3==target && dig.L3==target));
    
    motor_forward(0,0);
}
// Ajetaan seuraavaan risteykseen
void drive_to_intersection()
{
    // Ajan laskentaa kuuluvat muuttujat currentTime päivittää itsensä loopissa koko ajan ja elapsedTime asetetaan vain kerran
    // ja niiden välistä aikaa vertaillaan loopissa.
    int currentTime = 0;
    int elapsedTime = xTaskGetTickCount();
    // Sensorit
    struct sensors_ dig;
    // ajetaan niin kauan kunnes toinen puoli sensoreista näkee mustaa
    // siinä oletetaan olevan jonkin sortin risteys
    do {
        // tämän hetkinen aika päivitetään
        currentTime = xTaskGetTickCount();
        // vertaillaan currentTime ja elapsedTime välistä aikaa 1500 millisekunttiin
        // jos aika ylittää yli 1.5S niin rikotaan looppi koska oletetaan ettei tule enään
        // uutta risteystä
        if(currentTime - elapsedTime >= 1500){
            break;
        }
        motor_forward(40, 5);
        // Tehdään pieniä korjaus liikkeitä jos R2 tai L2 näkevät mustaa
        if(dig.R2 == BLACK){
            // Käännytään vähän oikealle (RIGHT)
		    SetMotors(0, 1, 50, 50, 2);
        }
        if(dig.L2 == BLACK){
            // Käännytään vähän vasemmalle (LEFT)
            SetMotors(1, 0, 50, 50, 2);
        }
        // sensoreiden päivitys
        reflectance_digital(&dig);
        // pyöritään niin kauan loopissa kunnes jompikumpi robotin sensori puolista näkee mustaa
        // ja oletetaan että ollaan jonkinsortin risteyksessä
    } while(! ((dig.R3==BLACK && dig.R2==BLACK) || ( dig.L2==BLACK && dig.L3==BLACK)));
    // vielä liikutaan risteyksen keskelle
	motor_forward(40,550);
    // ilmoitetaan risteyksen koordinaatit
    print_mqtt("Zumo09/position", "X: %d Y: %d", tank.positionX, tank.positionY);
    motor_forward(0,0);
}
// Robotin suunnan vaihto
void switch_tank_direction(){
    // tarkistetaan mihin suntaan robotti osoittaa
    // ja katsotaan kaikki mahdolliset risteys mahdollisuudet
    // koska jos esim robotti osoittaa ylös päin ja kääntyy vasemmalle
    // ja meillä ei ole reittiä vasemmalla vaan seuraava reitti on vasta
    // ala suunnassa asetetaan robotin suunta tällöin osoittamaan alas
    switch(tank.direction)
    {
        case UP:
            // robottin suunta on ylös ja 
            // jos robotti kääntyy vasemmalle ja ollaan risreyksessä jossa ei ole vasemman puoleista suuntaa
            // niin robotin tank_turn pysähtyy seuraavalle mustalle viivalle eli robotin suunta on fyysisesti
            // alas päin, joten asetetaan koodissa tank.direction alas päin (DOWN) jne...
            if(map[tank.positionY][tank.positionX] == LEFT_T_CROSS)
                tank.direction = DOWN;
            else if(map[tank.positionY][tank.positionX] == DOWN_LEFT_CORNER_CROSS)
                tank.direction = RIGHT;
            else if(map[tank.positionY][tank.positionX] == UP_LEFT_CORNER_CROSS)
                tank.direction = DOWN;
            else // FULL_CROSS || RIGHT_T_CROSS || DOWN_T_CROSS || DOWN_RIGHT_CORNER_CROSS || UP_RIGHT_CORNER_CROSS
                // Nyt havaitaan että ei ole enempää risteyksiä missä joudutaan hyppäämään vasemman suunnan yli
                // niin asetetaan tank.direction osoittamaan vasemmalle
                tank.direction = LEFT;
            break;
        // robotti osoittaa alaspäin ja käännytään vasemmalle, katsotaan kaikki mahdolliset hyppy kohdat
        case DOWN:
            if(map[tank.positionY][tank.positionX] == RIGHT_T_CROSS)
                tank.direction = UP;
            else if(map[tank.positionY][tank.positionX] == DOWN_RIGHT_CORNER_CROSS)
                tank.direction = UP;
            else if(map[tank.positionY][tank.positionX] == UP_RIGHT_CORNER_CROSS)
                tank.direction = LEFT;
            else // FULL_CROSS || LEFT_T_CROSS || DOWN_T_CROSS || DOWN_LEFT_CORNER_CROSS || UP_LEFT_CORNER_CROSS
                tank.direction = RIGHT;
            break;
        // robotti osoittaa vasemmalle ja käännytään vasemmalle, katsotaan kaikki mahdolliset hyppy kohdat
        case LEFT:
            if(map[tank.positionY][tank.positionX] == DOWN_T_CROSS)
                tank.direction = RIGHT;
            else if(map[tank.positionY][tank.positionX] == DOWN_LEFT_CORNER_CROSS)
                tank.direction = RIGHT;
            else if(map[tank.positionY][tank.positionX] == DOWN_RIGHT_CORNER_CROSS)
                tank.direction = UP;
            else // FULL_CROSS || RIGHT_T_CROSS || LEFT_T_CROSS || DOWN_RIGHT_CORNER_CROSS || UP_RIGHT_CORNER_CROSS
                tank.direction = DOWN;
            break;
        // robotti osoittaa oikealle ja käännytään vasemmalle, katsotaan kaikki mahdolliset hyppy kohdat
        case RIGHT:
            if(map[tank.positionY][tank.positionX] == UP_T_CROSS)
                tank.direction = LEFT;
            else if(map[tank.positionY][tank.positionX] == UP_LEFT_CORNER_CROSS)
                tank.direction = DOWN;
            else if(map[tank.positionY][tank.positionX] == UP_RIGHT_CORNER_CROSS)
                tank.direction = LEFT;
            else // FULL_CROSS || RIGHT_T_CROSS || LEFT_T_CROSS || DOWN_T_CROSS || DOWN_LEFT_CORNER_CROSS || DOWN_RIGHT_CORNER_CROSS
                tank.direction = UP;
            break;
        default:
        break;
    }
}
// Keijon tank_turn funktio
void tank_turn(){
    struct sensors_ dig;
    // laitetaan moottorit kääntymään eri suuntiin ja niin että käännös tapahtuu vasemmallle
    SetMotors(1, 0, 40, 40, 0);
    // käydään switch_tank_direction() funktiossa asettamassa oletettu tuleva robotin suunta
    switch_tank_direction();
    do{
        vTaskDelay(3);
        reflectance_digital(&dig);
        // pyöritään mustalta viivalta pois niin että molemmat keskimmäiset sensorit näkevät valkoista
    }while(!(dig.R1 == WHITE && dig.L1 == WHITE));
    do{
        vTaskDelay(3);
        reflectance_digital(&dig);
        // pyöritään niin kauan kunnes keskimmäiset sensorit näkevät mustaa
    }while(!(dig.R1 == BLACK && dig.L1 == BLACK));
    motor_forward(0,0);
}
// Skannaa kaikki risteyksen suunnat 
void scanpaths(){
    
    int distance = Ultra_GetDistance();
    int count = 0;
    // jos risteys ei ole täysi risteys niin lasketaan kyseisen risteyksen suunnat bittien avulla.
    if(map[tank.positionY][tank.positionX] < FULL_CROSS){
        // asetetaan kyseinen risteys temp muttujaan väliaikaisesti
        int temp = map[tank.positionY][tank.positionX];
        // pyöritään loopissa niin kauan kunnes temp muuttujalla ei ole arvoa eli kaikki bitit on 0
        while(temp){
            // lisätään temp muuttujasta ensimmäinen bitti and operaattorin avulla
            // eli jos eka bitti on 0 ja käytetään and 1 niin count muttujaan lisätään 0
            // tai jos eka bitti on 1 ja and 1 niin count muttujaan lisätään +1
            count += temp & 1;
            // siiretään bittejä 1 oikealle. 
            temp >>= 1;
        }
    }
    else{ // jos risteys on täysi niin risteyksessä on 4 suuntaa skannatavana
        count = 4;
    }
    // for loopataan kaikki risteyksen suunnat
    for(int i = 0; i < count; i++){
        // päivitetään etäisyys anturi
        distance = Ultra_GetDistance();
        // jos matka johonkin esteeseen on alle tai yhtä suuri kuin 12cm niin seuraavassa risteyksessä on
        // todennäköisesti este
        if(distance <= 12){
            // tarkistetaan robotin suunta
            switch(tank.direction)
            {
                // jos robotti osoittaa ylös niin robotin yläpuolella on este
                case UP:
                    // vaihdetaan map muuttujassa risteys pois käytöstä
                    //map[tank.positionY + 2][tank.positionX] &= ~(1 << DOWN);
                    map[tank.positionY + 1][tank.positionX] = NO_CROSS;
                    //map[tank.positionY][tank.positionX] &= ~(1 << UP);
                    //map[tank.positionY + 1][tank.positionX - 1] &= ~(1 << RIGHT);
                    //map[tank.positionY + 1][tank.positionX + 1] &= ~(1 << LEFT);
                    break;
                // jos robotti osoittaa alas niin robotin alapuolella on este
                case DOWN:
                    // vaihdetaan map muuttujassa risteys pois käytöstä
                    //map[tank.positionY - 2][tank.positionX] &= ~(1 << UP);
                    map[tank.positionY - 1][tank.positionX] = NO_CROSS;
                    //map[tank.positionY][tank.positionX] &= ~(1 << DOWN);
                    //map[tank.positionY - 1][tank.positionX - 1] &= ~(1 << RIGHT);
                    //map[tank.positionY - 1][tank.positionX + 1] &= ~(1 << LEFT);
                    break;
                // jos robotti osoittaa vasemmalle niin robotin vasemmalla puolella on este
                case LEFT:
                    // vaihdetaan map muuttujassa risteys pois käytöstä
                    //map[tank.positionY][tank.positionX - 2] &= ~(1 << RIGHT);
                    map[tank.positionY][tank.positionX - 1] = NO_CROSS;
                    //map[tank.positionY][tank.positionX] &= ~(1 << LEFT);
                    //map[tank.positionY + 1][tank.positionX - 1] &= ~(1 << DOWN);
                    //map[tank.positionY - 1][tank.positionX - 1] &= ~(1 << UP);
                    break;
                // jos robotti osoittaa oikealle niin robotin oikealla puolella on este
                case RIGHT:
                    // vaihdetaan map muuttujassa risteys pois käytöstä
                    //map[tank.positionY][tank.positionX + 2] &= ~(1 << LEFT);
                    map[tank.positionY][tank.positionX + 1] = NO_CROSS;
                    //map[tank.positionY][tank.positionX] &= ~(1 << RIGHT);
                    //map[tank.positionY + 1][tank.positionX + 1] &= ~(1 << DOWN);
                    //map[tank.positionY - 1][tank.positionX + 1] &= ~(1 << UP);
                    break;
                default:
                break;
            }
        }
        // käännytään vasemmalle
        tank_turn();
    }
}
// asetetaan uusi väliaikainen maali robotille
void next_goal(int *posX, int *posY){
    // asetetaan maali robotin X akselille ja yksi ylemmäs niin robotti pystyy ajamaan suoraan jos ei ole sitten estettä
    *posX = tank.positionX;
    *posY = tank.positionY + 1;
    // jos map tiedossa on että kyseisessä kohtaa ei ole risteystä vaihdetaan maalin paikka satunnaisesti x akselilla
    while(map[*posY][*posX] == NO_CROSS){
        *posX = rand() % 7;
    }
	//printf("goal  X: %d, Y: %d\n",*posX, *posY);
}

void zmain(void)
{
	int stopTime = 0;
	int startTime = 0;
    // käynnistetään kaikki tarpeelliset sensorit ja moottorit
    Ultra_Start();
    motor_start();
	reflectance_start();
	IR_Start();
    
    // asetetaan robotin tiedot kohilleen.
    // oletetaan että robotti aloittaa kartan keskeltä ja alhaalta
    // robottin suunta myös joudutaan olettamaan olevan ylös päin
    tank.positionX = 3;
    tank.positionY = 0;
    tank.direction = UP;
    
    // sallitaan liikkuvuus risteykseen
    bool move = true;
    // asetetaan väliaikainen maali valmiiksi
    int next_goal_X = 3;
    int next_goal_Y = 2;
    
    // set center sensor threshold 
	reflectance_set_threshold(10000, 10000, 15000, 15000, 10000, 10000); 
    
	send_mqtt("Zumo09/Maze", "Boot");
	IR_flush(); // clear IR receive buffer
	
    // asetetaan satunnainen luku rand funktiota varten
	print_mqtt("Zumo09/button", "Waiting user button");
	while(SW1_Read()){
		vTaskDelay(5);
    }
    srand(xTaskGetTickCount());
    
    // ajetaan mustalle viivalle odottamaan ir signaalia
    drive_to(BLACK);
	send_mqtt("Zumo09/ready", "maze");
	IR_wait();
    // asetetaan aloitus aika ja tulostetaan se
    startTime = xTaskGetTickCount();
	print_mqtt("Zumo09/start", "Start Time: %d ms", startTime);
    
    // ajetaan pois mustalta viivalta ja sen jälkeen ensimmäiseen risteykseen
    drive_to(WHITE);
    drive_to_intersection();
    // pyöritään niin kauan kunnes robotti on ajanut 14 riville
	while(tank.positionY != 14){
        // jos väliaikainen maali on vasemmalla ja robotti ei osoita vasemmalle käännetään robottia niin kauan kunnnes se on kääntynyt vasemmalle
        if(next_goal_X < tank.positionX){
            while (tank.direction != LEFT){
                    tank_turn();
            }
        }
        // jos väliaikainen maali on oikealla ja robotti ei osoita oikealle käännetään robottia niin kauan kunnnes se on kääntynyt oikealle
        else if(next_goal_X > tank.positionX){
            while (tank.direction != RIGHT){
                    tank_turn();
            }
        }
        else{
        // jos väliaikainen maali on ylhäällä ja robotti ei osoita ylös käännetään robottia niin kauan kunnnes se on kääntynyt ylöspäin
        // ja alaspäin ei tarvitse kääntyä koskaan koska maali on aina yhden risteyksen verran robotia ylempänä
            while (tank.direction != UP){
                    tank_turn();
            }
        }
        // päivitetään sensori
        int distance = Ultra_GetDistance();
        // jos sensori mittaa jonkin esteen olevan alle 14 cm päässä niin robotti skannaa kaikki mahdolliset suunnat
        if(distance <= 14){
            scanpaths();
        }
        // jos robotti jostain syystä liikkuu yhden risteyksen alemmas niin lasketaan väliaikainen maali myös yhden alemmas
        if(next_goal_Y > tank.positionY + 1)
        {
            next_goal_Y--;
        }
        // jos skannauksessa käy ilmi että väliaikainen maali on esteen sisällä joudutaan maalin paikka vaihtamaan
        if(map[next_goal_Y][next_goal_X] == NO_CROSS){
            next_goal(&next_goal_X, &next_goal_Y);
        }
        // kun robotti saavuttaa väliaikaisen maalin asetetaan sille uusi paikka
        if(next_goal_X == tank.positionX && next_goal_Y == tank.positionY){
            next_goal(&next_goal_X, &next_goal_Y);
        }
        
        do{
            // tarkistetaan voidaanko liikkua robotin osoittamaan suuntaan
            switch(tank.direction)
            {
                case UP:
                    // jos robotin yläpuolella ei ole risteystä tai on este niin asetetaan move muuttuja falseksi. 
                    // (move muuttuja kertoo robottille että pitää vaihtaa suuntaa ja robotti satunnaisesti vaihtaa suuntaa)
                    if( map[tank.positionY + 1][tank.positionX] == NO_CROSS){
                        move = false;
                        //printf("Cannot move UP X: %d Y: %d tile_id: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    // jos robotin yläpuolella on ihan tavallinen risteys voidaan sinne suuntaan liikkua
                    // ja päivitetään robotin koordinaatti
                    else{
                        tank.positionY++;
                        move = true;
                       //printf("dir: UP X: %d Y: %d tile_id: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    break;
                case DOWN:
                    // jos robotin alapuolella ei ole risteystä tai on este niin asetetaan move muuttuja falseksi. 
                    if( map[tank.positionY - 1][tank.positionX] == NO_CROSS){
                        move = false;
                        //printf("Cannot move DOWN X: %d Y: %d tile_id: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    // jos robotin alapuolella on ihan tavallinen risteys voidaan sinne suuntaan liikkua
                    // ja päivitetään robotin koordinaatti
                    else{
                        tank.positionY--;
                        move = true;
                        //printf("dir: DOWN X: %d Y: %d tile_id: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    break;
                case LEFT:
                    // jos robotin vasemmalla puolella ei ole risteystä tai on este niin asetetaan move muuttuja falseksi. 
                    if(map[tank.positionY][tank.positionX - 1] == NO_CROSS){
                        move = false;
                        //printf("Cannot move LEFT X: %d Y: %d tile_id: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    // jos robotin vasemmalla puolella on ihan tavallinen risteys voidaan sinne suuntaan liikkua
                    // ja päivitetään robotin koordinaatti
                    else{
                        tank.positionX--;
                        move = true;
                        //printf("dir: LEFT X: %d Y: %d tile_id: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    break;
                case RIGHT:
                    // jos robotin oikealla puolella ei ole risteystä tai on este niin asetetaan move muuttuja falseksi. 
                    if(map[tank.positionY][tank.positionX + 1] == NO_CROSS){
                        move = false;
                        //printf("Cannot move RIGHT tile_id: %d X: %d Y: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    // jos robotin oikealla puolella on ihan tavallinen risteys voidaan sinne suuntaan liikkua
                    // ja päivitetään robotin koordinaatti
                    else
                    {
                        tank.positionX++;
                        move = true;
                        //printf("dir: RIGHT tile_id: %d X: %d Y: %d \r\n", tank.positionX,tank.positionY, map[tank.positionY][tank.positionX]);
                    }
                    break;
                default:
                break;
            }
            // jos robotti ei voi liikkua johonkin osoittamaan suuntaan niin pyöritetään robottia satunnaisesti ja
            // toivotaan että seuraavassa suunnassa ei ole estettä
            if(!move){
                for(int i = 0; i < rand() % 2 + 1; i++)
                tank_turn();
            }
            //pyöritään loopissa niin kauan kunnes todetaan että tähän suuntaan saa liikkua
        }while(!move);
        // jos sallitaan liikkuminen niin liikutaan seuraavaan risteykseen
        if(move){
            drive_to_intersection();
        }
		vTaskDelay(5);
	}
    stopTime = xTaskGetTickCount();
    motor_forward(0,0);
    // ilmoitetaan robotin alku, loppu ja suoritus aika 
	print_mqtt("Zumo09/start", "Start Time: %d ms", startTime);
	print_mqtt("Zumo09/stop", "Stop Time: %d ms", stopTime);
	print_mqtt("Zumo09/time", "Run Time: %d ms",  stopTime - startTime);
    while(true){
		vTaskDelay(5);
    }
}
#endif