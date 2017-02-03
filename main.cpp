#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "Addons.h"
#include "GeoMath.h"
#include "ErrorLog.h"
#include "UI_Buttons.h"
#include "UI_Labels.h"
#include "ParseCommand.h"
#include "MotorManager.h"

void addDystr(int* tabDys, float volume, float step){
    if(volume < -step*5)tabDys[0]++;
    if(volume < -step*4 && volume > -step*5)tabDys[1]++;
    if(volume < -step*3 && volume > -step*4)tabDys[2]++;
    if(volume < -step*2 && volume > -step*3)tabDys[3]++;
    if(volume < -step && volume > -step*2)tabDys[4]++;
    if(volume < step && volume > -step)tabDys[5]++;
    if(volume < step*2 && volume > step)tabDys[6]++;
    if(volume < step*3 && volume > step*2)tabDys[7]++;
    if(volume < step*4 && volume > step*3)tabDys[8]++;
    if(volume < step*5 && volume > step*4)tabDys[9]++;
    if(volume > step*6)tabDys[1]++;
}

void printfDys(int* tabDys){
    for(int i=9; i>-1; i--){
        for(int j=0; j<11; j++){
            if(tabDys[j]>i)
                printf("_");
            else
                printf(" ");
        }
        printf("|\n");
    }
}

void test_movment(){
    float def1 = 10;
    int def2 = 0;
    float def3 = pow(5, 5)/100;
    float def4 = 1;
    
    int* dys_z1;
    int* dys_z2;
    int* dys_z3;
    int* dys_z4;
    int* dys_z5;
    dys_z1 = new int[11];
    dys_z2 = new int[11];
    dys_z3 = new int[11];
    dys_z4 = new int[11];
    dys_z5 = new int[11];
    for(int i=0; i<11; i++)dys_z1[i]=0;
    for(int i=0; i<11; i++)dys_z2[i]=0;
    for(int i=0; i<11; i++)dys_z3[i]=0;
    for(int i=0; i<11; i++)dys_z4[i]=0;
    for(int i=0; i<11; i++)dys_z5[i]=0;
    
    float er_min_z1 = 0;
    float er_max_z1 = 0;
    float er_mid_z1 = 0;
    float er_min_z2 = 0;
    float er_max_z2 = 0;
    float er_mid_z2 = 0;
    float er_min_z3 = 0;
    float er_max_z3 = 0;
    float er_mid_z3 = 0;
    float er_min_z4 = 0;
    float er_max_z4 = 0;
    float er_mid_z4 = 0;
    float er_min_z5 = 0;
    float er_max_z5 = 0;
    float er_mid_z5 = 0;
    float t_max = 0;
    float t_min = 0;
    float t_mid = 0;
    
    /*platform.*/printf("\r\n  \r\n  \r\n  \r\n>>>  RUN calculation: joints --> cartesian --> joints  <<<\r\n");
    
    for(float z1 = -def1; z1< def1+1; z1+=5){
    for(float z2 = -def1; z2< def1+1; z2+=5){
    for(float z3 = -def1; z3< def1+1; z3+=5){
    for(float z4 = -def1; z4< def1+1; z4+=5){
    for(float z5 = -def1; z5< def1+1; z5+=5){
    
    printf("%f %d <-> %f %f %f %f %f\t", (float)def2/def3, def2, z1, z2, z3, z4, z5);
    def2++;
    
    int t0,t1,t2;
    float zt1, zt2, zt3, zt4, zt5;
    t2 = 0;
    Coordinates j_temp(jointsCo, deg2rad(z1),deg2rad(z2),deg2rad(z3),deg2rad(z4),deg2rad(z5));
    ///*platform.*/printf("User's input of joints: j1=%f, j2=%f, j3=%f, j5=%f, j6=%f\r\n",rad2deg(j_temp.k1),rad2deg(j_temp.k2),rad2deg(j_temp.k3),rad2deg(j_temp.k4),rad2deg(j_temp.k5));
    t0=sys.getRefTime();
    Coordinates ca_temp=joints2cartes(j_temp);
    t1=sys.getRefTime();
    //t2 += t1 - t0;
    ///*platform.*/printf("Calculated cartesianCo: x=%f, y=%f, z=%f, A=%f, B=%f \r\n",ca_temp.k1,ca_temp.k2,ca_temp.k3,rad2deg(ca_temp.k4),rad2deg(ca_temp.k5));
    ///*platform.*/printf("Time of calculation: %d ms\r\n",t1-t0);
    LED1.toggle();
    t0=sys.getRefTime();
    Coordinates j_temp2=cartes2joints(ca_temp, Coordinates(jointsCo, deg2rad(z1-def4),deg2rad(z2-def4),deg2rad(z3-def4),deg2rad(z4-def4),deg2rad(z5-def4)), 0.1);
    t1=sys.getRefTime();
    t2 += t1 - t0;
    ///*platform.*/printf("Calculated jointsCo: j1=%f, j2=%f, j3=%f, j5=%f, j6=%f\r\n",rad2deg(j_temp2.k1),rad2deg(j_temp2.k2),rad2deg(j_temp2.k3),rad2deg(j_temp2.k4),rad2deg(j_temp2.k5));
    ///*platform.*/printf("Time of calculation: %d ms\r\n",t1-t0);
    t0=sys.getRefTime();
    Coordinates ca_temp2=joints2cartes(j_temp2);
    t1=sys.getRefTime();
    t2 += t1 - t0;
    
    Coordinates x_temp(jointsCo, deg2rad(z1-def4),deg2rad(z2-def4),deg2rad(z3-def4),deg2rad(z4-def4),deg2rad(z5-def4));
    Coordinates x_temp1=joints2cartes(x_temp);
    
    float dys = sqrt(pow((x_temp1.k1-ca_temp.k1),2)+pow((x_temp1.k2-ca_temp.k2),2)+pow((x_temp1.k3-ca_temp.k3),2));
    printf(" %f\n", dys);
    ///*platform.*/printf("Calculated cartesianCo: x=%f, y=%f, z=%f, A=%f, B=%f \r\n",ca_temp.k1,ca_temp.k2,ca_temp.k3,rad2deg(ca_temp.k4),rad2deg(ca_temp.k5));
    ///*platform.*/printf(">>> DONE AND NOkING ELSE TO DO <<<\r\n");
    if(t2 > t_max)t_max = t2;
    if(t2 < t_min)t_min = t2;
    t_mid += t2;
    addDystr(dys_z1, ca_temp.k1-ca_temp2.k1, 0.005);
    addDystr(dys_z2, ca_temp.k2-ca_temp2.k2, 0.005);
    addDystr(dys_z3, ca_temp.k3-ca_temp2.k3, 0.005);
    addDystr(dys_z4, ca_temp.k4-ca_temp2.k4, 0.005);
    addDystr(dys_z5, ca_temp.k5-ca_temp2.k5, 0.005);
    zt1 = abs(ca_temp.k1-ca_temp2.k1);
    zt2 = abs(ca_temp.k2-ca_temp2.k2);
    zt3 = abs(ca_temp.k3-ca_temp2.k3);
    zt4 = abs(ca_temp.k4-ca_temp2.k4);
    zt5 = abs(ca_temp.k5-ca_temp2.k5);
    if(er_max_z1<zt1)er_max_z1 = zt1;
    if(er_min_z1>zt1)er_min_z1 = zt1;
    er_mid_z1 += zt1;
    if(er_max_z2<zt2)er_max_z2 = zt2;
    if(er_min_z2>zt2)er_min_z2 = zt2;
    er_mid_z2 += zt2;
    if(er_max_z3<zt3)er_max_z3 = zt3;
    if(er_min_z3>zt3)er_min_z3 = zt3;
    er_mid_z3 += zt3;
    if(er_max_z4<zt4)er_max_z4 = zt4;
    if(er_min_z4>zt4)er_min_z4 = zt4;
    er_mid_z4 += zt4;
    if(er_max_z5<zt5)er_max_z5 = zt5;
    if(er_min_z5>zt5)er_min_z5 = zt5;
    er_mid_z5 += zt5;
    }}}}}
    
    t_mid = t_mid/def2;
    er_mid_z1 = er_mid_z1/def2;
    er_mid_z2 = er_mid_z2/def2;
    er_mid_z3 = er_mid_z3/def2;
    er_mid_z4 = er_mid_z4/def2;
    er_mid_z5 = er_mid_z5/def2;
    
    printf("czas regulacji => min %f, max %f, mid %f\t\n", t_min, t_max, t_mid);
    printf("blad regulacji x => min %f, max %f, mid %f\t\n", er_min_z1, er_max_z1, er_mid_z1);
    printf("blad regulacji y => min %f, max %f, mid %f\t\n", er_min_z2, er_max_z2, er_mid_z2);
    printf("blad regulacji z => min %f, max %f, mid %f\t\n", er_min_z3, er_max_z3, er_mid_z3);
    printf("blad regulacji A => min %f, max %f, mid %f\t\n", er_min_z4, er_max_z4, er_mid_z4);
    printf("blad regulacji B => min %f, max %f, mid %f\t\n", er_min_z5, er_max_z5, er_mid_z5);
    
    for(int i=0; i<11; i++){dys_z1[i]=dys_z1[i]/50;}
    for(int i=0; i<11; i++){dys_z2[i]=dys_z2[i]/50;}
    for(int i=0; i<11; i++){dys_z3[i]=dys_z3[i]/50;}
    for(int i=0; i<11; i++){dys_z4[i]=dys_z4[i]/50;}
    for(int i=0; i<11; i++){dys_z5[i]=dys_z5[i]/50;}
    printfDys(dys_z1);
    printf("\n\n");
    printfDys(dys_z2);
    printf("\n\n");
    printfDys(dys_z3);
    printf("\n\n");
    printfDys(dys_z4);
    printf("\n\n");
    printfDys(dys_z5);
    
    printf(">>> DONE AND NOkING ELSE TO DO <<<\r\n");
    
}

void hMain()
{
    Serial.init(115200);//inicjalizacja pedkoci seriala
    sys.setSysLogDev(&devNull);//wylaczenie wypisywania na serial i ui komend z systemu
    sys.taskCreate(printfErrorTask);//task wywołujacy wypisywanie danych na serial -> ErrorLog.h
    sys.taskCreate(ComandInputTask);//task odczytu komend z seriala ->ParseCommand.h
    platform.begin(&RPi);
    platform.ui.configHandler = cfgHandler;
    //platform.ui.onButtonEvent = onButtonEvent;
    platform.ui.setProjectId("@@@PROJECT_ID@@@");
    sys.setLogDev(&Serial);//ustawienie wypisywania na Serial
    
    //sys.taskCreate(MotorManagerUpdateTask;//task updatujący silniki
    
    for(int i=0;i<9;i++){
        LED1.toggle();
        sys.delay(1000);
    }
    
    //test_movment();
    
    for (;;)
    {
        sys.delay(500);
        LED2.toggle();
    }
}


