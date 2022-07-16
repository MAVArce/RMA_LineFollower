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
    int leftMotorHandle = 0;
    float vLeft = 0;
    int rightMotorHandle = 0;
    int cameraLinhaHandler = 0;
    float vRight = 0;
    string sensorNome[16];
    int sensorHandle[16];

    // variaveis de cena e movimentação do pioneer
    float v0 = 2;

    int clientID = simxStart((simxChar *)serverIP.c_str(), serverPort, true, true, 2000, 5);

    if (clientID != -1) {
        cout << "Servidor conectado!" << std::endl;

        // inicialização dos motores
        if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/leftMotor", (simxInt *)&leftMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok) {
            cout << "Handle do motor esquerdo nao encontrado!" << std::endl;
        }
        else {
            cout << "Conectado ao motor esquerdo!" << std::endl;
        }

        if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/rightMotor", (simxInt *)&rightMotorHandle, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok) {
            cout << "Handle do motor direito nao encontrado!" << std::endl;
        }
        else {
            cout << "Conectado ao motor direito!" << std::endl;
        }

        if (simxGetObjectHandle(clientID, (const simxChar *)"/PioneerP3DX/CameraLinha", (simxInt *)&cameraLinhaHandler, (simxInt)simx_opmode_oneshot_wait) != simx_return_ok)
            cout << "Handle da camera de linha nao encontrado!" << std::endl;
        else
            cout << "Conectado a camera de linha!" << std::endl;
        cv::namedWindow("LinhaCamera", cv::WINDOW_AUTOSIZE );
        cv::Mat image;
        image = cv::imread("./imagemTest.png", 1 );
        float* auxValues=nullptr;
        int* auxValuesCount=nullptr;
        simxInt *resolution = (simxInt*)calloc(2,sizeof(simxInt));
        simxUChar *imageResult = (simxUChar*)calloc(256*256*3, sizeof(simxUChar));

        simxReadVisionSensor(clientID, cameraLinhaHandler, nullptr, nullptr, nullptr, simx_opmode_streaming);
        simxGetVisionSensorImage(clientID, cameraLinhaHandler, resolution, &imageResult, 0, simx_opmode_streaming);
        std::cout << "resolution[0]: " << resolution[0] << ", resolution[1]: " << resolution[1] << std::endl;
        // desvio e velocidade do robô
        while (simxGetConnectionId(clientID) != -1){ // enquanto a simulação estiver ativa
            vLeft = 0.0;
            vRight = 0.0;

            // atualiza velocidades dos motores
            simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat)vLeft, simx_opmode_streaming);
            simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat)vRight, simx_opmode_streaming);
            simxGetVisionSensorImage(clientID, cameraLinhaHandler, resolution, &imageResult, 0, simx_opmode_streaming);
            

            // espera um pouco antes de reiniciar a leitura dos sensores
            // extApi_sleepMs(5);
            if(resolution[0] > 0){
                cv::Mat my_mat(256,256,CV_8UC3,&imageResult[0]);
                cv::imshow("LinhaCamera", my_mat);
            }
            // Press  ESC on keyboard to exit
            char c = (char)cv::waitKey(5);
            if (c == 27)
                break;
        }

        simxFinish(clientID); // fechando conexao com o servidor
        cout << "Conexao fechada!" << std::endl;
    }
    else {
        cout << "Problemas para conectar o servidor!" << std::endl;
    }

    return 0;
}
