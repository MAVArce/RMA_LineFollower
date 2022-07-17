/*
    Client of V-REP simulation server (remoteApi)
    Copyright (C) 2015  Rafael Alceste Berri rafaelberri@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Habilite o server antes na simulação V-REP com o comando lua:
// simExtRemoteApiStart(portNumber) -- inicia servidor remoteAPI do V-REP

extern "C"
{
#include "remoteApi/extApi.h"
}

// #include "./include/v_repLib.h"
#include <iostream>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "include/vision.hpp"
#include "include/control.hpp"
#include "include/colorSearch.hpp"

using namespace std;

int main(int argc, char **argv) {
    constexpr double pi = 3.14159265358979323846;
    
    int scene = 1; //1 - cena normal, 2 - pezinho, 3 - papeis coloridos
    string serverIP = "127.0.0.1";

    if(argc > 2){
        try{
            scene = stoi(string(argv[2]));

            if(scene != 2 && scene != 3){
                scene = 1;
            }

        } catch(exception &err){
            cout << "Escreva uma cena válida (1 ou 2)." << endl;
            cout << "Considerando cena padrão" << endl;
        }
    }

    if (argc > 1) {
        char *ip = argv[1];
        serverIP = string(ip);
    }

    int serverPort = 19999;
    int clientID = simxStart((simxChar *)serverIP.c_str(), serverPort, true, true, 2000, 5);

    if (clientID == -1) {
        cout << "Problemas para conectar o servidor!" << std::endl;
        return -1;
    }
    cout << "Servidor conectado!" << std::endl;

    int robotHandle = 0;
    int leftMotorHandle = 0;
    int rightMotorHandle = 0;
    
    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX", (simxInt *)&robotHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok){
        cout << "Robô nao encontrado!" << std::endl;
    } else {
        cout << "Conectado ao robô!" << std::endl;
    }

    // inicialização dos motores
    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/leftMotor", (simxInt *)&leftMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok){
        cout << "Handle do motor esquerdo nao encontrado!" << std::endl;
    } else {
        cout << "Conectado ao motor esquerdo!" << std::endl;
    }

    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/rightMotor", (simxInt *)&rightMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok) {
        cout << "Handle do motor direito nao encontrado!" << std::endl;
    } else {
        cout << "Conectado ao motor direito!" << std::endl;
    }

    simxFloat pos[3];
    simxFloat ang[3];

    if(scene == 1){
        pos[0] = -1.7;
        pos[1] = 1.0;
        pos[2] = 0.1388;
        ang[0] = 0.0;
        ang[1] = 0.0;
        ang[2] = 0.0;
    } else if(scene == 2){
        pos[0] = 14.8305;
        pos[1] = 0.725;
        pos[2] = 0.1388;
        ang[0] = 0.0;
        ang[1] = 0.0;
        ang[2] = 55.0 * pi / 180;
    } else if(scene == 3){
        pos[0] = -1.613;
        pos[1] =  1.016;
        pos[2] = 0.1388;
        ang[0] = 0.0;
        ang[1] = 0.0;
        ang[2] = -179.60 * pi / 180;
    }

    // // simxFloat pos[3] = {-0.1766, -0.4299, 0.1388}; //pezinho 
    simxSetObjectPosition(clientID, robotHandle, -1, pos, (simxInt)simx_opmode_oneshot);
    simxSetObjectOrientation(clientID, robotHandle, -1, ang, (simxInt)simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)0, simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)0, simx_opmode_streaming);

    int cameraLinhaHandler = 0;
    int cameraFrontalHandler = 0;

    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/CameraLinha", (simxInt *)&cameraLinhaHandler, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
        cout << "Handle da camera de linha nao encontrado!" << std::endl;
    else
        cout << "Conectado a camera de linha!" << std::endl;

    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/CameraFrontal", (simxInt *)&cameraFrontalHandler, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
        cout << "Handle da camera de Frontal nao encontrado!" << std::endl;
    else
        cout << "Conectado a camera de Frontal!" << std::endl;

    Vision visionCtrl = Vision(clientID, cameraLinhaHandler, cameraFrontalHandler);
    Actuator actuator = Actuator(clientID, leftMotorHandle, rightMotorHandle);
    ColorSearch colorSearch = ColorSearch(clientID, robotHandle, &visionCtrl);

    cv::namedWindow("CameraLinha", cv::WINDOW_AUTOSIZE );
    cv::namedWindow("CameraFrontal", cv::WINDOW_AUTOSIZE );
    cv::Mat image;

    int last_sim_time = 0;
    int curr_sim_time = 0;
    int dt = 0;

    // Control distCtrl(0.001f, 0.0f, 0.0002f);
    // Control angleCtrl(0.5f, 0.0f, 0.2f);
    // float v0 = 1;

    Control distCtrl(0.007f, 0.00001f, 0.0015f);
    Control angleCtrl(0.7f, 0.0001f, 0.06f);
    float v0 = 1.5;
    float vLeft = 0;
    float vRight = 0;
    float angle, dist;

    colorSearch.Calibrate(&actuator);

    cv::Point ptLandmark;
    int distLandmark;

    // desvio e velocidade do robô
    while (simxGetConnectionId(clientID) != -1) {// enquanto a simulação estiver ativa 
        curr_sim_time = (int)simxGetLastCmdTime(clientID);
        dt = curr_sim_time - last_sim_time;
        last_sim_time = curr_sim_time;

        cv::Mat linhaImg = visionCtrl.getImageLinha();
        if(dt == 0){
            extApi_sleepMs(1);
            continue;   
        }

        // espera um pouco antes de reiniciar a leitura dos sensores
        cv::Mat grey, gauss, thres;
        cv::cvtColor(linhaImg, grey, cv::COLOR_RGB2GRAY);
        cv::GaussianBlur(grey, gauss, cv::Size(5,5), 0, 0, 0);
        cv::threshold(gauss, thres, 60, 255, cv::THRESH_BINARY_INV);

        // cv::imshow("CameraLinha", linhaImg);
        // cv::imshow("CameraLinha gauss", gauss);
        // cv::imshow("CameraLinha thres", thres);

        vector<cv::Point> contour = visionCtrl.getBiggestContour(thres);

        if(!contour.empty())
        {
            drawContours(linhaImg, vector<vector<cv::Point> >(1,contour), -1, cv::Scalar(0,0,255), 2, cv::LINE_8);

            cv::Vec4f line4f;
            fitLine(contour, line4f, cv::DIST_L2, 0, 0.01, 0.01);
            cv::Point pt1;
            cv::Point pt2;

            int bottomx = int(((linhaImg.rows - line4f[3]) * line4f[0] / line4f[1]) + line4f[2]);

            if(abs(line4f[0]) > 0.01){
                int lefty = int((-line4f[2]*line4f[1]/line4f[0]) + line4f[3]); // -x1 * a + y1
                int righty = int(((linhaImg.cols-line4f[2])*line4f[1]/line4f[0])+line4f[3]); // (x2 - x1) * a + y1
                pt1 = cv::Point(linhaImg.cols-1,righty);
                pt2 = cv::Point(0,lefty);
                angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x);

                if(angle < -pi/2){
                    angle += pi/2;
                } else if (angle > pi/2){
                    angle -= pi/2;
                }
            } else {
                int topx = int(- (line4f[3] * line4f[0] / line4f[1]) + line4f[2]);
                
                pt1 = cv::Point(bottomx, linhaImg.rows-1);
                pt2 = cv::Point(topx, 0);
                angle = 0.0f;
            }

            dist = isnan(bottomx)? 0.0 : ((linhaImg.cols - 1) / 2 - bottomx);
            cv::line(linhaImg, pt1, pt2, cv::Scalar(0,255,0), 2);

        }
        cv::imshow("CameraLinha", linhaImg);
        
        vLeft = v0;
        vRight = v0;

        std::cout << "Distancia: " << dist << std::endl;
        distCtrl.updateVelocities(-dist, vLeft, vRight, dt);
        std::cout << "Angulo: " << angle << std::endl;
        angleCtrl.updateVelocities(angle, vLeft, vRight, dt);
        std::cout << "VLeft: " << vLeft << std::endl;
        std::cout << "VRight: " << vRight << std::endl;
        std::cout << std::endl;

        actuator.sendVelocities(vLeft, vRight);


        cv::Mat frontalImg = visionCtrl.getImageFrontal();
        cv::imshow("CameraFrontal", frontalImg);
        // cv::Mat frontalImgCopy = frontalImg.clone();
        // colorSearch.FindLandmark(&ptLandmark, &distLandmark, &frontalImgCopy);
        // cv::imshow("CameraFrontal", frontalImgCopy);

        // Press  ESC on keyboard to exit
        char c = (char)cv::waitKey(15);
        if (c == 27)
            break;
    }

    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;

    return 0;
}
