# 🦾 reBot-DevArm : bras robotique open source pour tous les développeurs

<p align="center">
  <img src="./media/RS5_56.png" alt="Bannière reBot-DevArm">
</p>

<p align="center">
    <a href="https://certification.oshwa.org/cn000024.html">
  <img src="./media/certification-mark-CN000024-wide.png" width="180">
</a>
</p>

<p align="center">
    <a href="./LICENSE">
    <img src="https://img.shields.io/badge/License-CERN--OHL--W--2.0--for--hardware-green.svg" alt="License: CERN-OHL-W-2.0">
    </a>
    <img src="https://img.shields.io/badge/License-Apache--2.0--for--software-pink.svg" alt="License: Apache-2.0">
    </a>
    <img src="https://img.shields.io/badge/Commercial-Contact%20Us-red.svg" alt="yaohui.zhu@seeed.cc">
    <img src="https://img.shields.io/badge/ROS-Noetic%20%7C%20Humble-orange.svg" alt="ROS Support">
    <img src="https://img.shields.io/badge/Framework-LeRobot-yellow.svg" alt="LeRobot">
    <img src="https://img.shields.io/badge/Framework-Isaac Sim-yellow.svg" alt="LeRobot">
</p>

<p align="center">
  <strong>🚀 100 % open source · IA incarnée · Stack complet matériel + logiciel</strong>
</p>

<p align="center">
  <strong>📦 Construisez votre propre bras robotique · 🧠 Apprenez la robotique · 🏭 Déployez des applications réelles</strong>
</p>

<table align="center">
  <tr>
    <td>
      <a href="https://www.youtube.com/watch?v=ONbpv3seiG8">
        <img src="https://img.icons8.com/ios-filled/100/ff0000/youtube-play.png" width="40">
      </a>
    </td>
    <td>
      <a href="https://www.youtube.com/watch?v=ONbpv3seiG8">
        About The reBot Arm
      </a>
    </td>
  </tr>
</table>

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a> &nbsp;|&nbsp;
    <a href="./README_JP.md">日本語</a>&nbsp;|&nbsp;
    <a href="./README_Fr.md">français</a>&nbsp;|&nbsp;
    <a href="./README_es.md">Español</a>
  </strong>
</p>

<p align="center">
<a href="https://discord.gg/AbGuqJhDpQ">
    <img src="https://img.shields.io/discord/1409155673572249672?color=7289DA&label=Discord&logo=discord&logoColor=white"></a>
<a href="https://wiki.seeedstudio.com/robotics_page/">  
    <img src="https://img.shields.io/badge/Documentation-📕-blue" alt="wiki robotique"></a>
</p>

## 📖 Introduction

**reBot-DevArm (reBot Arm B601 DM et reBot Arm B601 RS)** est un projet de bras robotique dédié à réduire les barrières d’apprentissage de l’IA incarnée. Nous mettons l’accent sur le **« véritable open source »** — pas seulement le code, nous ouvrons absolument tout sans réserve :
- 🦾 **Deux versions du bras robotique**：Nous fournirons tous les fichiers open source pour deux versions du bras robotique ayant la même apparence : **Robostride** et **Damiao**.
- 🛠️ **Plans matériels** : fichiers sources pour les pièces en tôle et les pièces imprimées en 3D.
- 🔩 **Liste BOM** : détails complets jusqu’aux spécifications et aux liens d’achat de chaque vis.
- 💻 **Logiciels & algorithmes** : SDK Python, ROS1/2, Isaac Sim, LeRobot, etc.

## Obtenez votre propre bras robotique reBot Arm

- Nous proposons cinq options de kits sur [Seeedstudio.com](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) :
  - **Kit moteurs du corps du bras** : comprend uniquement les moteurs et les faisceaux de câblage du bras robotique.
  - **Kit structure du corps du bras** : comprend uniquement les composants structurels mécaniques.
  - **Kit complet de préhenseur** : comprend les moteurs, les faisceaux de câblage et les composants structurels du préhenseur.
  - **Kit complet** : comprend l'ensemble complet du corps du bras robotique et du préhenseur.
  - **Bras robotique préassemblé** : bras robotique fini et entièrement assemblé.

- Le kit Seeedstudio ne comprend pas d'adaptateur d'alimentation ni de serre-joints en C comme accessoires standard. Cette configuration tient compte du fait que les utilisateurs peuvent alimenter l'unité avec des batteries ou la monter sur une base DIY personnalisée. Vous pouvez acheter séparément une [alimentation](https://www.seeedstudio.com/AC-DC-Power-Adapter-IEC-60320-C14-XT30-Female-24V-4-5A-1200mm-L190-W92-5-H36mm-p-6764.html) et un [cordon d'alimentation](https://www.seeedstudio.com/reServer-AC-US-p-5052.html), ou consulter la solution d'alimentation Mean Well indiquée en bas de notre [BOM](./hardware/reBot_B601_DM/readme.md/#about-power-supply).

- Vous pouvez également acheter le [Leader Arm](https://www.seeedstudio.com/Star-Arm-102-p-6765.html?qid=P2U7IG_yskyak5m_1776415593315) et l'[alimentation 12V 10A](https://www.seeedstudio.com/FY1209900-12V-10A-Power-Adapter-12V-10A-p-6496.html). Vous pouvez aussi utiliser l'adaptateur d'alimentation 12 V CC du SO-ARM101 pour alimenter le Leader.

-------------------
- Pour la version reBot Arm RS, nous proposons deux options de kits sur [Seeedstudio.com](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) :
  - **Kit complet** : comprend l'ensemble complet non assemblé du corps du bras robotique et du préhenseur.
  - **Bras robotique préassemblé** : bras robotique fini et entièrement assemblé.

- Nous recommandons fortement d'utiliser l'alimentation [Meanwell 48V 12.5A](https://www.amazon.com/sspa/click?ie=UTF8&spc=MTo0NzgzODk2NzUxNTQ0NzEyOjE3ODE2MTA2NTU6c3BfYXRmOjIwMDExNjA5NjQwMTc5ODo6MDo6&url=%2FLRS-350-48-Price-Switching-Supply-MeanWell%2Fdp%2FB0BP6S5DYR%2Fref%3Dsr_1_1_sspa%3Fcrid%3D27VPQOWNPN9UG%26dib%3DeyJ2IjoiMSJ9.qK84sGJa4-74kbCEX11MOFBju8sSQUdFsbHw6PNvmaEHnhzjX2T7dyhRNJY01mXxpWk8lccGOwnezxmqLKUjqglX_FI26mrxlvZf0KNiLdJ8QnhKsber4KDoyyLHNxWGV451uHCzZbCDXxM0iYXVnubuVourRaRURlyMorRavuLd2a32kABx-BKqyF5Dfr7dV453ecE6QULFqG-UVLBaBRijbxQGTJ2YiNyXAqn3bkM.Bt5mAPOJNAWGnXCC2mwvjdDdccZd1_0-WRXZpP4mR4M%26dib_tag%3Dse%26keywords%3DLRS-350-48%26qid%3D1781610655%26sprefix%3Dlrs-350-%252Caps%252C331%26sr%3D8-1-spons%26sp_csd%3Dd2lkZ2V0TmFtZT1zcF9hdGY%26psc%3D1) pour le modèle RS. Si vous avez besoin de plus de puissance pour libérer toutes ses performances, vous pouvez opter pour un adaptateur d'alimentation 48V 25A.
------------------


## 🗺️ Feuille de route & état

Nous nous engageons à maintenir et à adapter en continu les principaux écosystèmes de développement robotique. Voici notre état actuel d’adaptation et le calendrier de publication prévu :

### reBot Arm B601 DM
| Écosystème pris en charge | État | Description / date de publication estimée | Documentation associée |
| :--- | :---: | :--- | :--- |
| **Utilisation de base des moteurs** | ✅ Terminé | Contrôle de mouvement de base et encapsulation d'API | [Damiao Technology](https://wiki.seeedstudio.com/cn/damiao_series/) |
| **Open source des nouvelles pièces structurelles STEP 3D et de la BOM** | ✅ Terminé | Fichiers STEP de toutes les pièces de la nouvelle version, BOM des pièces et prix de référence de toutes les pièces usinées | [reBot Arm B601-DM BOM](./hardware/reBot_B601_DM/readme.md) |
| **Référence pour les tests de performance sur machine réelle** | ✅ Terminé | Référence de performance du bras robotique dans des conditions de fonctionnement normales et extrêmes | [Performance Testing](./hardware/reBot_B601_DM/performance_testing/Performance_Testing.md) |
| **Vidéo d'assemblage** | ✅ Terminé | Étapes d'assemblage ultra détaillées et vidéo | [Getting Started with reBot Arm B601-DM](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/) |
| **SDK Python** | ✅ Optimisation continue, PR bienvenues | Intégration tout-en-un de la lecture, de l'écriture et du contrôle des moteurs Robstride, Damiao, Mota, Gaoqing, Hexfellow et autres. | [Getting Started with Motorbridge](https://motorbridge.seeedstudio.com) and [Web UI](https://rebot-devarm.w0x7ce.eu/) |
| **Intégration ROS2** | ✅ Terminé | Contrôleur reBot Arm intégré à ROS2 avec prise en charge de la cinématique, de la planification de trajectoire et de la compensation gravitationnelle | [reBot Arm B601-DM ROS2 Integration Guide](https://wiki.seeedstudio.com/rebot_arm_b601_dm_ros2_integration/) |
| **Intégration Pinocchio** | ✅ Terminé | Adaptation au framework Pinocchio, permettant la cinématique directe/inverse et la compensation gravitationnelle du bras robotique | [Getting Started with Pinocchio for reBot Arm B601-DM](https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/) and [Github repo](https://github.com/vectorBH6/reBotArm_control_py) |
| **Simulation Isaac Sim** | 🚧 En cours | Importation de modèles USD et activation de la téléopération simulée | [delay for add additional courses: 2026.06.20] |
| **Intégration LeRobot** | ✅ Terminé | Adaptation au framework d'entraînement Hugging Face LeRobot | [Getting Started with LeRobot-based reBot Arm](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) |
| **Intégration caméra de profondeur** | ✅ Terminé | Démonstration de préhension visuelle basée sur YOLO et caméra de profondeur | [Getting Started with Visual Grasping Demo](https://wiki.seeedstudio.com/rebot_arm_b601_dm_grasping_demo/) |
| **Intégration vocale reSpeaker** | ✅ Terminé | Ajout du réseau reSpeaker Flex à 4 micros pour construire un système de contrôle intelligent du bras robotique piloté par la voix avec conscience spatiale | [reBot Arm B601-DM Voice Control](https://wiki.seeedstudio.com/control_rebot_arm_using_voice_with_respeaker_flex/) |
| **Mises à jour progressives des derniers algorithmes** | ⏳ Planifié | Les algorithmes grand public seront mis à jour progressivement | Ongoing |
| **Lancement d'une série de cours entièrement gratuits** | ⏳ Planifié | Les algorithmes grand public seront mis à jour progressivement | Ongoing |

#### Contributions des développeurs
| Écosystème pris en charge | Auteurs | Description / date de publication estimée | Documentation ou dépôt associé |
| :--- | :---: | :--- | :--- |
| **ROS2 (Humble), intégration third_party, URDF / rebotarm_bringup** | [@danieldoradotalaveron-rb](https://github.com/danieldoradotalaveron-rb) | 1. **Moniteur de diagnostics passif** (`rebotarm_monitor_ros2`) : superposition `/diagnostics` pour `rqt_robot_monitor`, agrégateur compatible serial/CAN ;<br>2. **Stationnement et arrêt sûrs** : capture de la pose de repos à la connexion, retour lent à l'arrêt ou via `/rebotarm/park` pour éviter une chute soudaine ;<br>3. **Compensation de gravité (arrêt doux)** : sortie progressive MIT lors de la sortie de compensation de gravité pour éliminer les claquements, secousses et instabilités pendant le passage pos/vel ;<br>4. **Téléopération gamepad avec IK/FK et mesures de sécurité** : contrôle de l'effecteur via IK, visualisation en direct de l'état du robot dans RViz (test simulation uniquement) ;<br>5. **TF D405 eye-in-hand** : configuration Xacro sous `end_link` dans `rebotarm_bringup` pour visualisation RViz et TF uniquement (sans driver/profondeur/intrinsèques). Pose de montage ajustable via le fichier launch, calibration du support non terminée. Teleop FK/IK utilise l'URDF `fixend_core` bras seul, xacro complet pour RSP/RViz. | [rebotarm_monitor_ros2](https://github.com/danieldoradotalaveron-rb/rebotarm_monitor_ros2)、[reBotArmController_ROS2](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2) |

### reBot Arm B601 RS

| Écosystème pris en charge | État | Description / date de publication estimée | Documentation associée |
| :--- | :---: | :--- | :--- |
| **Utilisation de base des moteurs** | ✅ Terminé | Contrôle de mouvement de base et encapsulation d’API | [Robstride](https://wiki.seeedstudio.com/cn/robstride_control/) |
| **Open source des nouvelles pièces structurelles STEP 3D et de la BOM** | 🚧 En cours | Fichiers STEP de toutes les pièces de la nouvelle version, BOM des pièces et prix de référence de toutes les pièces usinées | Prévu [2026.05] |
| **Vidéo d’assemblage** | 🚧 En cours | Étapes d’assemblage ultra détaillées et vidéo | [Prévu 2026.05] |
| **ROS2 (Humble)** | ⏳ Planifié | Les pilotes principaux sont terminés, et MoveIt2 est actuellement en cours d’optimisation | [Prévu 2026.05] |
| **Intégration LeRobot** | ⏳ Planifié | Adaptation au framework d’entraînement Hugging Face LeRobot | [Prévu 2026.05] |
| **Intégration Pinocchio** | ✅ Terminé | Adaptation au framework Pinocchio, permettant la cinématique directe/inverse et la compensation gravitationnelle pour le bras robotique | [Getting Started with Pinocchio for reBot Arm B601-DM](https://wiki.seeedstudio.com/rebot_arm_b601_rs_pinocchio_meshcat/) ainsi que [Github code de contrôle](https://github.com/vectorBH6/reBotArm_control_py) |
| **Simulation Isaac Sim** | ⏳ Planifié | Importation de modèles USD et activation de la téléopération simulée | [Prévu 2026.05] |
| **Mises à jour progressives des derniers algorithmes** | ⏳ Planifié | Les algorithmes grand public seront mis à jour progressivement | En continu |
| **Lancement d’une série de cours entièrement gratuits** | ⏳ Planifié | Les algorithmes grand public seront mis à jour progressivement | En continu |


---

## ⚙️ Spécifications matérielles

reBot-DevArm est conçu pour des applications d’IA incarnée sur bureau, en équilibrant la capacité de charge utile et la flexibilité.

| Paramètre | reBot Arm B601-DM | reBot Arm B601-RS |
| :--- | :--- | :--- |
| **Charge utile (Payload)** | 1,5 kg | **2,5 kg** |
| **Espace de travail recommandé** | 70 % de l’espace de travail de portée du bras | 70 % de l’espace de travail de portée du bras |
| **Portée maximale (Reach)** | 767 mm | **754 mm** |
| **Poids (Weight)** | **Env. 4,5 kg** | Env. 6,7 kg |
| **Répétabilité** | < 0,2 mm | < 0,2 mm |
| **Degrés de liberté (DOF)** | 6 DOF + 1 pince | 6 DOF + 1 pince |
| **Plateformes/écosystèmes pris en charge** | ROS1, ROS2, LeRobot, Pinocchio, Isaac Sim, SDK Python | ROS1, ROS2, LeRobot, Pinocchio, Isaac Sim, SDK Python |
| **Tension d’alimentation** | DC 24V | DC 48V |

----

## Retours de la communauté
| <img src="/community/GEM-4.png" height="100"> | <img src="/community/from_Linyan.png" height="100">   |<img src="/community/from_Diddi.png" height="100">  |<img src="/community/from_Henderson.jpg" height="100">  | <img src="/community/from_Sameer.png" height="100">|
| --- | --- | --- | --- |  --- |
| [From GEM-4: Gemma Embodied 4 Physical Assistance](https://www.kaggle.com/competitions/gemma-4-good-hackathon/writeups/new-writeup-1778618527713) | [From Linyan Fu](https://x.com/Linyan_Fu/status/2056383947341525180)  and [Apheth D Almeida](https://x.com/Apheth_DAlmeida/status/2053503164507476096)| [From Dhruv Diddi](https://x.com/DhruvDiddi/status/2046605015008383284)  | [From Ed Henderson](https://x.com/ed0henderson/status/2055076839002095743)  | From Sameer |
| <img src="/community/from_Binh_Pham.png" height="100"> | <img src="/community/from_fangtianchonghui.png" height="100">   |<img src="/community/from_xensedyl.png" height="100">  |<img src="/community/from_Henderson_2.png" height="100">  | |
| [From Binh_Pham](https://x.com/pham_blnh/status/2061994096374505710) | [From FangTianChongHui](https://www.instagram.com/reel/DY7Ny8OPjVu/?utm_source=ig_web_copy_link&igsh=NTc4MTIwNjQ2YQ==)| [Xense YaoLin Dong](https://x.com/dong1505lin)  | [From Ed Henderson](https://x.com/ed0henderson/status/2055076839002095743)  | |

## 🧹 Accessoires optionnels
### Support de caméra au poignet
| UVC 32×32 | Intel D435i | Intel D405 et Gemini 305 | Gemini 2 |
| --- | --- | --- | --- |
| <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/UVC_camera_mount.png" height="100"> | <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/D435i.jpg" height="100"> |  <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/D405.jpg" height="100"> | <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/Gemini2.jpg" height="100"> |
| [STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/UVC32_mount.step) | [STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/D435_Gemini2_Mount.step) | [STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/D405_305_Mount.step) |[STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/D435_Gemini2_Mount.step) |

### Compatible avec le bras Leader
| Star Arm 102-LD | Ouvert à l'intégration et la compatibilité |
| --- | --- |
|  <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/star_arm_102.jpg" height="100">  | Bientôt disponible |
|  [Dépôt Github](https://github.com/servodevelop/Star-Arm-102) | Bientôt disponible |


### Doigt souple DIY
| Doigt souple | Intégration compatible ouverte |
| --- | --- |
|  <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/Soft_Finger.png" height="100">  |Bientôt disponible|
| [Support de doigt (ABS/PLA)](/hardware/reBot_B601_DM/3D_Printed_Parts/Soft_Gripper_Mount.step) et [Doigt (TPU 95+)](/hardware/reBot_B601_DM/3D_Printed_Parts/Soft_Gripper_Finger.step)  |Bientôt disponible |

---

### 🎓 Écosystème robotique full-stack
reBot-DevArm n’est pas seulement un bras robotique, mais une communauté d’apprentissage de la robotique. Nous partageons gratuitement les tutoriels généraux suivants :

#### 🖥️ Edge Computing & contrôle principal
*   [![Jetson](https://img.shields.io/badge/NVIDIA-reComputer%20Jetson-76B900?style=for-the-badge&logo=nvidia&logoColor=white)](https://wiki.seeedstudio.com/NVIDIA_Jetson/) —— **Inférence IA & cœur de calcul**
*   [![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-4B%20%2F%205-C51A4A?style=for-the-badge&logo=Raspberry%20Pi&logoColor=white)](https://wiki.seeedstudio.com/raspberry-pi-devices/) —— **Environnement général de développement Linux**
*   [![ESP32](https://img.shields.io/badge/MCU-Seeed%20XIAO%20(ESP32)-0091BD?style=for-the-badge&logo=espressif&logoColor=white)](https://wiki.seeedstudio.com/SeeedStudio_XIAO_Series_Introduction/) —— **Nœud de contrôle sans fil basse consommation**

#### 📡 Capteurs & périphériques
*   **🚗 Moteurs & servomoteurs** : [Damiao / Gogo / Robstride / Mita / Feite / Fashion Star](https://wiki.seeedstudio.com/robotics_page/)
*   **👁️ Perception visuelle** : [Caméras de profondeur / LiDAR / algorithmes de vision](https://wiki.seeedstudio.com/robotics_page/)
*   **👂 Interaction auditive** : [reSpeaker Mic Arrays/Voice Control/Spatial Awareness(DoA)](https://wiki.seeedstudio.com/control_rebot_arm_using_voice_with_respeaker_flex/)
*   **🧭 Mouvement & attitude** : [IMU (6 axes/9 axes) / gyroscopes / magnétomètres](https://wiki.seeedstudio.com/Sensor/IMU/)
*   **🤖 Kits complets** : [Plus de capteurs robotiques & d’exemples de pilotes](https://wiki.seeedstudio.com/robotics_page/)


> 👉 **[Cliquez pour accéder à la base de connaissances du Wiki](https://wiki.seeedstudio.com/)** (Tous les tutoriels sont consultables gratuitement)

---


## 🙌 Références & remerciements
Le chemin de l’open source n’est jamais solitaire. La naissance du projet reBot-DevArm n’aurait pas été possible sans le soutien total de Seeed Studio, de la communauté open source mondiale et d’excellents partenaires matériels. Nous exprimons notre plus profond respect aux projets et équipes suivants :

### 🌍 Écosystème & support logiciel
*   **[Seeed Studio](https://www.seeedstudio.com/)** - Fournit un support complet en chaîne d’approvisionnement matériel et en assistance technique.
*   **[Hugging Face LeRobot](https://github.com/huggingface/lerobot)** - Un excellent framework d’apprentissage robotique de bout en bout.
*   **[NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim)** - Une puissante plateforme de simulation robotique et de données synthétiques.

### ⚙️ Partenaires matériels principaux
Merci aux fabricants suivants pour avoir fourni des solutions de moteurs et d’actionneurs hautes performances :
*   **[Damiao Technology](https://www.damiaokeji.com/)**
*   **[Robstride](https://robstride.com/)**
*   **[Fashion Star](https://fashionstar.com.hk/wiki/)**

### 💡 Inspiration
Ce projet est profondément inspiré par les excellents projets open source suivants :
*   **[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/main)**
*   **[Mobile ALOHA](https://github.com/tonyzhaozh/aloha)**
*   **[Dummy-Robot (Zhihui Jun)](https://github.com/peng-zhihui/Dummy-Robot)**
*   **[OpenArm](https://openarm.dev/)**
*   **[I2RT](https://i2rt.com/)**
*   **[TRLC-DK1](https://github.com/robot-learning-co/trlc-dk1)**

### 🎃 Contributeurs du prototype
- **Équipe SeeedStudio AI Robotics** : Yaohui Zhu (yaohui.zhu@seeed.cc)
- **SeeedStudio STU** : Wentao Dong
- **SeeedStudio STU** : Weiwei Xu
- **Département des achats de SeeedStudio** : Fengqun Peng


### 👥 Contributeurs

## Nos principaux contributeurs 
<p align="center"><a href="https://github.com/Seeed-Projects/reBot-DevArm/graphs/contributors">
  <img src="https://contributors-img.web.app/image?repo=Seeed-Projects/reBot-DevArm" />
</a></p>



*Bientôt disponible... N’hésitez pas à soumettre des PR pour devenir contributeur !*

## Historique des étoiles

[![Star History Chart](https://api.star-history.com/svg?repos=Seeed-Projects/reBot-DevArm&type=date&legend=top-left)](https://www.star-history.com/#Seeed-Projects/reBot-DevArm&type=date&legend=top-left)

# Licence du projet reBot-DevArm

- **Conception matérielle** © 2026 Seeed Studio Co., Ltd. (SeeedStudio), publié sous licence [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt)
- **Code du firmware** © 2026 Seeed Studio Co., Ltd. (SeeedStudio), publié sous licence [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)

## Droits et restrictions

Chers développeurs et experts du secteur, le projet de bras robotique reBot Arm a toujours adhéré aux valeurs fondamentales d'**Agilité, d'Ouverture, de Responsabilité et de Symbiose** au service de la communauté des développeurs. Notre vision est de permettre à chaque passionné de maîtriser systématiquement l'architecture matérielle et les principes logiciels des bras robotiques, et de vivre une expérience immersive avec les algorithmes de pointe de l'intelligence incarnée, grâce au projet reBot.

Pendant les cinq premiers mois suivant son lancement, le projet a utilisé la licence open source **CC BY-SA NC (Non-Commercial)**. L'intention initiale était de permettre à tous les développeurs et contributeurs de se concentrer sur l'itération et l'amélioration du produit pendant sa phase initiale, moins mature, sans être perturbés par des préoccupations commerciales, et de se consacrer pleinement à la co-construction et à l'optimisation du projet.

Après des mois de perfectionnement approfondi du produit et de maturation technique par Seeed Studio, **à compter du 11 mai 2026**, le projet reBot Arm est officiellement passé de la licence CC BY-SA NC à la licence open source **CERN-OHL-W 2.0**.

À partir de ce moment, le projet atteint une **open source à 100 % sur l'ensemble de la chaîne (matériel et logiciel)** , accordant **des droits d'utilisation commerciale complets pour tous les scénarios**.

Nous espérons que vous continuerez à participer, dans un esprit d'inclusion et de collaboration, à soutenir, maintenir et approfondir la communauté open source reBot Arm, à partager les fruits de l'open source et à construire ensemble un écosystème pour l'intelligence incarnée.

Ce projet utilise différentes licences open source pour le matériel et le logiciel. Veuillez confirmer les termes de la licence applicables à la partie que vous utilisez.

| Élément / Licence                          | Matériel reBot : CERN-OHL-W-2.0                              | SDK logiciel reBot : Apache-2.0                              |
| ------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| **✅ Utilisation commerciale autorisée**   | ✅ Autorisée                                                 | ✅ Autorisée                                                 |
| **✅ Modification autorisée**              | ✅ Autorisée                                                 | ✅ Autorisée                                                 |
| **✅ Redistribution autorisée**            | ✅ Autorisée                                                 | ✅ Autorisée                                                 |
| **✅ Intégration/redistribution en source fermée** | ❌ Conditionnelle (voir [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt) pour plus de détails) | ✅ Autorisée (aucune obligation de divulguer le code modifié) |
| **⚠️ Conservation de la mention de copyright requise** | ✅ Requise                                                   | ✅ Requise                                                   |
| **⚠️ Conservation du texte de la licence requise** | ✅ Requise                                                   | ✅ Requise                                                   |
| **⚠️ Mention des modifications requise**  | ✅ Requise (avec date et description)                        | ✅ Requise (avec description des modifications)              |
| **⚠️ Licence de brevet**                   | ✅ Licence de brevet explicite (voir [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt) pour plus de détails) | ✅ Licence de brevet explicite                               |
| **⚠️ Fourniture des sources lors de la distribution** | ✅ **Obligation** de fournir les "Sources Complètes" du matériel | ❌ Aucune obligation de fournir les sources                  |
| **⚠️ Compatibilité avec les modules externes/fermés** | ✅ Autorisée (caractéristique Weakly Reciprocal)            | ✅ Totalement autorisée                                      |
| **🔗 Relation avec d'autres composants/modules** | Les modules d'interface indépendants (External Material) peuvent conserver leur licence d'origine (fermée) | Aucune restriction, peut se lier à des bibliothèques sous n'importe quelle licence |
| **📄 Texte officiel de la licence**        | [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt)        | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)    |


## ☎ Contactez-nous
- **Progrès open source & support technique**-Yaohui : yaohui.zhu@seeed.cc
- **Collaboration future & personnalisation**-Elaine : elaine.wu@seeed.cc
