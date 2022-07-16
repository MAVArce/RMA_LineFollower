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
#include <opencv2/opencv.hpp>

#include "include/control.hpp"

using namespace std;

int main(int argc, char **argv)
{
    string serverIP;

    if (argc > 1) {
        char *ip = argv[1];
        serverIP = string(ip);
    }
    else {
        serverIP = "127.0.0.1";
    }

    int serverPort = 19999;
    int clientID = simxStart((simxChar *)serverIP.c_str(), serverPort, true, true, 2000, 5);

    if (clientID == -1) {
        cout << "Problemas para conectar o servidor!" << std::endl;
        return -1;
    }
    cout << "Servidor conectado!" << std::endl;

    int robotHandle = 0;
    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX", (simxInt *)&robotHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok){
        cout << "Robô nao encontrado!" << std::endl;
    } else {
        cout << "Conectado ao robô!" << std::endl;
    }

    simxFloat pos[3] = {-1.7195, 0.9750, 0.1388}; //Initial pose
    // simxFloat pos[3] = {-0.1766, -0.4299, 0.1388}; //pezinho 
    simxFloat angle[3] = {0.0, 0.0, 0.0};
    simxSetObjectPosition(clientID, robotHandle, -1, pos, (simxInt)simx_opmode_oneshot);
    simxSetObjectOrientation(clientID, robotHandle, -1, angle, (simxInt)simx_opmode_oneshot);

    int leftMotorHandle = 0;
    int rightMotorHandle = 0;
    int cameraLinhaHandler = 0;
    int cameraFrontalHandler = 0;

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

    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/CameraLinha", (simxInt *)&cameraLinhaHandler, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
        cout << "Handle da camera de linha nao encontrado!" << std::endl;
    else
        cout << "Conectado a camera de linha!" << std::endl;

    if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/CameraFrontal", (simxInt *)&cameraFrontalHandler, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
        cout << "Handle da camera de Frontal nao encontrado!" << std::endl;
    else
        cout << "Conectado a camera de Frontal!" << std::endl;

    Control ctrl;

    float vLeft = 0;
    float vRight = 0;
    cv::namedWindow("CameraLinha", cv::WINDOW_AUTOSIZE );
    cv::namedWindow("CameraFrontal", cv::WINDOW_AUTOSIZE );
    cv::Mat image;
    simxInt *resolutionLinha = (simxInt*)calloc(2,sizeof(simxInt));
    simxUChar *imageLinhaResult = nullptr;
    simxInt *resolutionFrontal = (simxInt*)calloc(2,sizeof(simxInt));
    simxUChar *imageFrontalResult = nullptr;

    simxGetVisionSensorImage(clientID, cameraLinhaHandler, resolutionLinha, &imageLinhaResult, 0, simx_opmode_streaming);
    simxGetVisionSensorImage(clientID, cameraFrontalHandler, resolutionFrontal, &imageFrontalResult, 0, simx_opmode_streaming);

    // desvio e velocidade do robô
    while (simxGetConnectionId(clientID) != -1) {// enquanto a simulação estiver ativa 
        
        // ctrl.updateVelocities(0.0, 0.0, vLeft, vRight);
        vLeft = 0.0;
        vRight = 0.0;
        // atualiza velocidades dos motores
        simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);
        simxGetVisionSensorImage(clientID, cameraLinhaHandler, resolutionLinha, &imageLinhaResult, 0, simx_opmode_streaming);
        
        // espera um pouco antes de reiniciar a leitura dos sensores
        // extApi_sleepMs(5);
        if(resolutionLinha[0] > 0){
            cv::Mat grey(resolutionLinha[1],resolutionLinha[0],CV_8UC1);
            cv::Mat my_mat(resolutionLinha[1],resolutionLinha[0],CV_8UC3,&imageLinhaResult[0]);
            cv::flip(my_mat, my_mat, 0);

            cv::cvtColor(my_mat, grey, cv::COLOR_RGB2GRAY);

            cv::Mat gauss(resolutionLinha[1],resolutionLinha[0],CV_8UC1);
            cv::Mat thres(resolutionLinha[1],resolutionLinha[0],CV_8UC1);
            cv::GaussianBlur(grey, gauss, cv::Size(5,5), 0, 0, 0);
            cv::threshold(gauss, thres, 60, 255, cv::THRESH_BINARY_INV);

            vector<vector<cv::Point> > contours;
            vector<cv::Vec4i> hierarchy;
            cv::findContours(thres, contours, hierarchy, 1, cv::CHAIN_APPROX_NONE);

            int largest_contour_index = -1;
            double largest_area = 0;
            for (int i = 0; i< contours.size(); i++) // iterate through each contour. 
            {
                double a = cv::contourArea(contours[i], false);  //  Find the area of contour
                if (a>largest_area){
                    largest_area = a;
                    largest_contour_index = i;                //Store the index of largest contour
                }

            }

            if(largest_contour_index >= 0)
            {
                drawContours( my_mat, contours, largest_contour_index, cv::Scalar(0,0,255), 2, cv::LINE_8, hierarchy, 0 );

                cv::Vec4f line4f;
                fitLine(contours[largest_contour_index], line4f, cv::DIST_L2, 0, 0.01, 0.01);
                int lefty = int((-line4f[2]*line4f[1]/line4f[0]) + line4f[3]);
                int righty = int(((resolutionLinha[0]-line4f[2])*line4f[1]/line4f[0])+line4f[3]);
                cv::line(my_mat, cv::Point(resolutionLinha[0]-1,righty), cv::Point(0, lefty), cv::Scalar(0,255,0), 2);
            }
            cv::imshow("CameraLinha", my_mat);
        }

        simxGetVisionSensorImage(clientID, cameraFrontalHandler, resolutionFrontal, &imageFrontalResult, 0, simx_opmode_streaming);
        if(resolutionFrontal[0] > 0){
            cv::Mat my_mat2(resolutionFrontal[1],resolutionFrontal[0],CV_8UC3,&imageFrontalResult[0]);
            cv::flip(my_mat2, my_mat2, 0);
            cv::cvtColor(my_mat2, my_mat2, cv::COLOR_RGB2BGR);
            cv::imshow("CameraFrontal", my_mat2);
        }
        // Press  ESC on keyboard to exit
        char c = (char)cv::waitKey(5);
        if (c == 27)
            break;
    }

    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;

    return 0;
}
