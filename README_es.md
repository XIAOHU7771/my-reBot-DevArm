# 🦾 reBot-DevArm: Brazo Robótico de Código Abierto para Todos los Desarrolladores

<p align="center">
  <img src="./media/RS5_56.png" alt="Banner de reBot-DevArm">
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
  <strong>🚀 100% código abierto · IA corpórea · Stack completo de hardware + software</strong>
</p>

<p align="center">
  <strong>📦 Construye tu propio brazo robótico · 🧠 Aprende robótica · 🏭 Despliega aplicaciones reales</strong>
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
    <img src="https://img.shields.io/badge/Documentation-📕-blue" alt="wiki de robótica"></a>
</p>


## 📖 Introducción

**reBot-DevArm (reBot Arm B601 DM y reBot Arm B601 RS)** es un proyecto de brazo robótico dedicado a reducir la barrera de aprendizaje de la IA Corpórea. Nos enfocamos en el **"Verdadero Código Abierto"** — no solo el código, abrimos todo sin reservas:

* 🦾 **Dos versiones del brazo robótico**：Proporcionaremos todos los archivos de código abierto para dos versiones del brazo robótico con la misma apariencia: **Robostride** y **Damiao**。
* 🛠️ **Planos de hardware**: Archivos fuente de piezas de chapa metálica y piezas impresas en 3D。
* 🔩 **Lista BOM**: Detallada hasta las especificaciones y enlaces de compra de cada tornillo。
* 💻 **Software y algoritmos**: Python SDK, ROS1/2, Isaac Sim, LeRobot, etc。

## Consigue tu propio brazo reBot Arm

- Ofrecemos cinco opciones de kit en [Seeedstudio.com](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html):
  - **Kit de motores del cuerpo del brazo**: incluye solo motores y arneses de cableado para el brazo robótico.
  - **Kit estructural del cuerpo del brazo**: incluye solo componentes mecánicos estructurales.
  - **Kit completo de pinza**: incluye motores, arneses de cableado y componentes estructurales de la pinza.
  - **Kit completo**: incluye el conjunto completo del cuerpo del brazo robótico y la pinza.
  - **Brazo robótico preensamblado**: brazo robótico terminado y completamente ensamblado.

- El kit de Seeedstudio no incluye adaptador de corriente ni abrazaderas en C como accesorios estándar. Esta decisión tiene en cuenta que los usuarios pueden alimentar la unidad con baterías o montarla sobre una base DIY personalizada. Puede comprar por separado una [fuente de alimentación](https://www.seeedstudio.com/AC-DC-Power-Adapter-IEC-60320-C14-XT30-Female-24V-4-5A-1200mm-L190-W92-5-H36mm-p-6764.html) y un [cable de alimentación](https://www.seeedstudio.com/reServer-AC-US-p-5052.html), o consultar la solución de alimentación Mean Well mostrada al final de nuestra [BOM](./hardware/reBot_B601_DM/readme.md/#about-power-supply).

- También puede comprar el [Leader Arm](https://www.seeedstudio.com/Star-Arm-102-p-6765.html?qid=P2U7IG_yskyak5m_1776415593315) y la [fuente de alimentación 12V 10A](https://www.seeedstudio.com/FY1209900-12V-10A-Power-Adapter-12V-10A-p-6496.html). También puede usar el adaptador de corriente de 12 V CC del SO-ARM101 para alimentar el Leader.

-------------------
- Para la versión reBot Arm RS, ofrecemos dos opciones de kit en [Seeedstudio.com](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html):
  - **Kit completo**: incluye el conjunto completo sin ensamblar del cuerpo del brazo robótico y la pinza.
  - **Brazo robótico preensamblado**: brazo robótico terminado y completamente ensamblado.

- Recomendamos encarecidamente usar la fuente de alimentación [Meanwell 48V 12.5A](https://www.amazon.com/sspa/click?ie=UTF8&spc=MTo0NzgzODk2NzUxNTQ0NzEyOjE3ODE2MTA2NTU6c3BfYXRmOjIwMDExNjA5NjQwMTc5ODo6MDo6&url=%2FLRS-350-48-Price-Switching-Supply-MeanWell%2Fdp%2FB0BP6S5DYR%2Fref%3Dsr_1_1_sspa%3Fcrid%3D27VPQOWNPN9UG%26dib%3DeyJ2IjoiMSJ9.qK84sGJa4-74kbCEX11MOFBju8sSQUdFsbHw6PNvmaEHnhzjX2T7dyhRNJY01mXxpWk8lccGOwnezxmqLKUjqglX_FI26mrxlvZf0KNiLdJ8QnhKsber4KDoyyLHNxWGV451uHCzZbCDXxM0iYXVnubuVourRaRURlyMorRavuLd2a32kABx-BKqyF5Dfr7dV453ecE6QULFqG-UVLBaBRijbxQGTJ2YiNyXAqn3bkM.Bt5mAPOJNAWGnXCC2mwvjdDdccZd1_0-WRXZpP4mR4M%26dib_tag%3Dse%26keywords%3DLRS-350-48%26qid%3D1781610655%26sprefix%3Dlrs-350-%252Caps%252C331%26sr%3D8-1-spons%26sp_csd%3Dd2lkZ2V0TmFtZT1zcF9hdGY%26psc%3D1) para el modelo RS. Si necesita más potencia para desbloquear todo su rendimiento, puede optar por un adaptador de corriente de 48V 25A.
------------------


## 🗺️ Hoja de ruta y estado

Estamos comprometidos a mantener y adaptarnos continuamente a los ecosistemas principales de desarrollo robótico. A continuación se muestra nuestro progreso actual de adaptación y el plan de lanzamiento:

### reBot Arm B601 DM
| Ecosistema compatible | Estado | Descripción / Fecha estimada de lanzamiento | Documentación relacionada |
| :--- | :---: | :--- | :--- |
| **Uso básico del motor** | ✅ Completado | Control básico de movimiento y encapsulación de API | [Damiao Technology](https://wiki.seeedstudio.com/cn/damiao_series/) |
| **Código abierto de las nuevas piezas estructurales 3D STEP y BOM** | ✅ Completado | Archivos STEP de todas las piezas de la nueva versión, BOM de piezas y precios de referencia de todos los componentes mecanizados | [reBot Arm B601-DM BOM](./hardware/reBot_B601_DM/readme.md) |
| **Referencia de pruebas de rendimiento en máquina real** | ✅ Completado | Referencia de rendimiento del brazo robótico en condiciones normales y extremas de operación | [Performance Testing](./hardware/reBot_B601_DM/performance_testing/Performance_Testing.md) |
| **Video de ensamblaje** | ✅ Completado | Pasos de ensamblaje ultra detallados y video | [Getting Started with reBot Arm B601-DM](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/) |
| **Python SDK** | ✅ Optimización continua, PR bienvenidos | Integración integral de lectura/escritura y control de motores Robstride, Damiao, Mota, Gaoqing, Hexfellow y otros. | [Getting Started with Motorbridge](https://motorbridge.seeedstudio.com) and [Web UI](https://rebot-devarm.w0x7ce.eu/) |
| **Integración ROS2** | ✅ Completado | Controlador integrado ROS2 para reBot Arm con cinemática, planificación de trayectorias y compensación de gravedad | [reBot Arm B601-DM ROS2 Integration Guide](https://wiki.seeedstudio.com/rebot_arm_b601_dm_ros2_integration/) |
| **Integración Pinocchio** | ✅ Completado | Adaptación al framework Pinocchio, que permite cinemática directa/inversa y compensación de gravedad para el brazo robótico | [Getting Started with Pinocchio for reBot Arm B601-DM](https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/) and [Github repo](https://github.com/vectorBH6/reBotArm_control_py) |
| **Simulación Isaac Sim** | 🚧 En progreso | Importación de modelos USD y teleoperación simulada | [delay for add additional courses: 2026.06.20] |
| **Integración LeRobot** | ✅ Completado | Adaptación al framework de entrenamiento Hugging Face LeRobot | [Getting Started with LeRobot-based reBot Arm](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) |
| **Integración con cámara de profundidad** | ✅ Completado | Demostración de agarre visual basada en YOLO y cámara de profundidad | [Getting Started with Visual Grasping Demo](https://wiki.seeedstudio.com/rebot_arm_b601_dm_grasping_demo/) |
| **Integración de voz reSpeaker** | ✅ Completado | Añade el arreglo reSpeaker Flex de 4 micrófonos para construir un sistema inteligente de control por voz del brazo robótico con conciencia espacial | [reBot Arm B601-DM Voice Control](https://wiki.seeedstudio.com/control_rebot_arm_using_voice_with_respeaker_flex/) |
| **Actualizaciones graduales de los últimos algoritmos** | ⏳ Planificado | Los algoritmos principales se actualizarán progresivamente | Ongoing |
| **Lanzamiento de una serie de cursos completamente gratuitos** | ⏳ Planificado | Los algoritmos principales se actualizarán progresivamente | Ongoing |

#### Contribuciones de desarrolladores
| Ecosistema compatible | Autores | Descripción / Fecha estimada de lanzamiento | Documentación o repositorio relacionado |
| :--- | :---: | :--- | :--- |
| **ROS2 (Humble), integración third_party, URDF / rebotarm_bringup** | [@danieldoradotalaveron-rb](https://github.com/danieldoradotalaveron-rb) | 1. **Monitor pasivo de diagnósticos** (`rebotarm_monitor_ros2`): superposición `/diagnostics` para `rqt_robot_monitor`, agregador compatible con serial/CAN;<br>2. **Estacionamiento y apagado seguros**: captura la pose de reposo al conectar, retorno lento al apagar o con `/rebotarm/park` para evitar caídas repentinas;<br>3. **Compensación de gravedad (parada suave)**: salida gradual MIT al abandonar la compensación de gravedad para eliminar golpes, tirones e inestabilidad durante el cambio pos/vel;<br>4. **Teleoperación con gamepad usando IK/FK y medidas de seguridad**: control del efector final mediante IK, visualización en vivo del estado del robot en RViz (prueba solo en simulación);<br>5. **TF D405 eye-in-hand**: configuración Xacro bajo `end_link` en `rebotarm_bringup` para visualización RViz y TF únicamente (sin driver/profundidad/intrínsecos). La pose de montaje se puede ajustar mediante el archivo launch, calibración del soporte no completada. Teleop FK/IK usa URDF `fixend_core` solo del brazo, xacro completo para RSP/RViz. | [rebotarm_monitor_ros2](https://github.com/danieldoradotalaveron-rb/rebotarm_monitor_ros2)、[reBotArmController_ROS2](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2) |

### reBot Arm B601 RS

| Ecosistema compatible                   |     Estado     | Descripción / Fecha estimada de lanzamiento | Documentación relacionada                                       |
| :-------------------------------------- | :------------: | :------------------------------------------ | :-------------------------------------------------------------- |
| **Uso básico del motor**                |  ✅ Completado  | Control básico y encapsulación de API       | [Robstride](https://wiki.seeedstudio.com/cn/robstride_control/) |
| **Código abierto de piezas STEP y BOM** | 🚧 En progreso | Archivos STEP, BOM y precios de referencia  | Expected [2026.05]                                           |
| **Video de ensamblaje**                 | 🚧 En progreso | Guía de ensamblaje detallada                | [Expected 2026.05]                                           |
| **ROS2 (Humble)**                       |  ⏳ Planificado | Drivers listos, optimización en curso       | [Expected 2026.05]                                              |
| **Integración con LeRobot**             |  ⏳ Planificado | Framework de aprendizaje robótico           | [Expected 2026.05]                                              |
| **Integración con Pinocchio**           |  ✅ Completado | Cinemática y compensación de gravedad       | [Introducción a Pinocchio y MeshCat para reBot Arm B601-DM](https://wiki.seeedstudio.com/es/rebot_arm_b601_rs_pinocchio_meshcat/) y [Github Código de control](https://github.com/vectorBH6/reBotArm_control_py)                                              |
| **Simulación en Isaac Sim**             |  ⏳ Planificado | Simulación robótica                         | [Expected 2026.05]                                              |
| **Actualización de algoritmos**         |  ⏳ Planificado | Actualizaciones continuas                   | Ongoing                                                         |
| **Cursos gratuitos**                    |  ⏳ Planificado | Cursos abiertos                             | Ongoing                                                         |

---

## ⚙️ Especificaciones de hardware

reBot-DevArm está diseñado para aplicaciones de IA Corpórea en escritorio, equilibrando capacidad de carga y flexibilidad.

| Parámetro | reBot Arm B601-DM | reBot Arm B601-RS |
| :--- | :--- | :--- |
| **Carga útil (Payload)** | 1.5 kg | **2.5 kg** |
| **Espacio de trabajo recomendado** | 70% del alcance del brazo | 70% del alcance del brazo |
| **Alcance máximo (Reach)** | 767 mm | **754 mm** |
| **Peso (Weight)** | **Aprox. 4.5 kg** | Aprox. 6.7 kg |
| **Repetibilidad** | < 0.2 mm | < 0.2 mm |
| **Grados de libertad (DOF)** | 6 DOF + 1 pinza | 6 DOF + 1 pinza |
| **Plataformas/ecosistemas compatibles** | ROS1, ROS2, LeRobot, Pinocchio, Isaac Sim, Python SDK | ROS1, ROS2, LeRobot, Pinocchio, Isaac Sim, Python SDK |
| **Voltaje de alimentación** | DC 24V | DC 48V |

----

## Comentarios de la comunidad
| <img src="/community/GEM-4.png" height="100"> | <img src="/community/from_Linyan.png" height="100">   |<img src="/community/from_Diddi.png" height="100">  |<img src="/community/from_Henderson.jpg" height="100">  | <img src="/community/from_Sameer.png" height="100">|
| --- | --- | --- | --- |  --- |
| [From GEM-4: Gemma Embodied 4 Physical Assistance](https://www.kaggle.com/competitions/gemma-4-good-hackathon/writeups/new-writeup-1778618527713) | [From Linyan Fu](https://x.com/Linyan_Fu/status/2056383947341525180)  and [Apheth D Almeida](https://x.com/Apheth_DAlmeida/status/2053503164507476096)| [From Dhruv Diddi](https://x.com/DhruvDiddi/status/2046605015008383284)  | [From Ed Henderson](https://x.com/ed0henderson/status/2055076839002095743)  | From Sameer |
| <img src="/community/from_Binh_Pham.png" height="100"> | <img src="/community/from_fangtianchonghui.png" height="100">   |<img src="/community/from_xensedyl.png" height="100">  |<img src="/community/from_Henderson_2.png" height="100">  | |
| [From Binh_Pham](https://x.com/pham_blnh/status/2061994096374505710) | [From FangTianChongHui](https://www.instagram.com/reel/DY7Ny8OPjVu/?utm_source=ig_web_copy_link&igsh=NTc4MTIwNjQ2YQ==)| [Xense YaoLin Dong](https://x.com/dong1505lin)  | [From Ed Henderson](https://x.com/ed0henderson/status/2055076839002095743)  | |

## 🧹 Hardware opcional
### Soporte de cámara de muñeca
| UVC 32×32 | Intel D435i | Intel D405 y Gemini 305 | Gemini 2 |
| --- | --- | --- | --- |
| <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/UVC_camera_mount.png" height="100"> | <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/D435i.jpg" height="100"> |  <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/D405.jpg" height="100"> | <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/Gemini2.jpg" height="100"> |
| [STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/UVC32_mount.step) | [STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/D435_Gemini2_Mount.step) | [STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/D405_305_Mount.step) |[STEP](/hardware/reBot_B601_DM/3D_Printed_Parts/D435_Gemini2_Mount.step) |

### Compatible con el brazo Leader
| Star Arm 102-LD | Abierto para integración y compatibilidad |
| --- | --- |
|  <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/star_arm_102.jpg" height="100">  | Próximamente disponible |
|  [Repositorio de Github](https://github.com/servodevelop/Star-Arm-102) | Próximamente disponible |

### Dedo blando DIY
| Dedo blando | Integración de compatibilidad abierta |
| --- | --- |
|  <img src="/hardware/reBot_B601_DM/3D_Printed_Parts/images/Soft_Finger.png" height="100">  |Próximamente|
| [Soporte de dedo (ABS/PLA)](/hardware/reBot_B601_DM/3D_Printed_Parts/Soft_Gripper_Mount.step) y [Dedo (TPU 95+)](/hardware/reBot_B601_DM/3D_Printed_Parts/Soft_Gripper_Finger.step)  |Próximamente |

### Hardware opcional recomendado
<table>
  <thead>
    <tr>
      <th>Categoría</th>
      <th>Tipo</th>
      <th>Vista previa</th>
      <th>Producto</th>
      <th>Enlace</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="8" align="center"><strong>Cámara</strong></td>
      <td rowspan="6" align="center">Cámara de profundidad</td>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/0/-/0-101090144--orbbec-gemini-2-3d-camera.jpg" alt="Orbbec Gemini 2 3D Camera" width="120"></td>
      <td><strong>Orbbec Gemini 2 3D Camera</strong></td>
      <td><a href="https://www.seeedstudio.com/Orbbec-Gemini-2-3D-Camera-p-6464.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/1/0/1000000774.png" alt="Orbbec Gemini 336 Depth Camera" width="120"></td>
      <td><strong>Orbbec Gemini 336 Depth Camera</strong></td>
      <td><a href="https://www.seeedstudio.com/Orbbec-Gemini-336-3D-Camera-3D-p-6662.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/1/-/1-100010971-orbbec-gemini-335-lg.jpg" alt="Orbbec Gemini 335LG 3D Camera" width="120"></td>
      <td><strong>Orbbec Gemini 335LG 3D Camera</strong></td>
      <td><a href="https://www.seeedstudio.com/Orbbec-Gemini-335LG-3D-Camera-p-6541.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/0/0/0035718.png" alt="SLAMTEC Aurora S, Integrated Spatial Perception Module" width="120"></td>
      <td><strong>SLAMTEC Aurora S</strong><br><sub>Integrated Spatial Perception Module</sub></td>
      <td><a href="https://www.seeedstudio.com/SLAMTEC-Aurora-S-AI-Integrated-Spatial-Perception-System-p-6669.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/i/n/intel_realsense_d435i_34_1_1.jpg" alt="Intel RealSense Depth Camera D435i" width="120"></td>
      <td><strong>Intel RealSense Depth Camera D435i</strong></td>
      <td><a href="https://www.seeedstudio.com/Intel-RealSense-Depth-Camera-D435i-p-4423.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/1/-/1-100000540-realsense-d405-3d-camera.jpg" alt="RealSense Depth Camera D405" width="120"></td>
      <td><strong>RealSense Depth Camera D405</strong></td>
      <td><a href="https://www.seeedstudio.com/RealSense-D405-3D-Camera-p-6758.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td rowspan="2" align="center">Cámara monocular</td>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/0/-/0-101090101-3mp-gmsl2-camera-module-190-degree.jpg" alt="Sensing SG3S-ISX031C-GMSL2F 3MP GMSL2 Camera" width="120"></td>
      <td><strong>Sensing SG3S-ISX031C-GMSL2F 3MP GMSL2 Camera</strong></td>
      <td><a href="https://www.seeedstudio.com/SG3S-ISX031C-GMSL2F-p-6245.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/s/2/s231.1.jpg" alt="ET-S231 Megapixel 120 Degree Wide-Angle 1080P USB Camera Module" width="120"></td>
      <td><strong>ET-S231 Megapixel 120 Degree Wide-Angle 1080P USB Camera Module</strong></td>
      <td><a href="https://www.seeedstudio.com/ET-S231-120-USB-Camera-p-6683.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td rowspan="2" align="center"><strong>Micrófono</strong></td>
      <td rowspan="2" align="center">Matriz de micrófonos</td>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/1/-/1-100070894-respeaker-flex-xvf3800-circular-4-with-xiao-esp32s3_1_.jpg" alt="reSpeaker Flex XVF3800 Circular-4 with XIAO ESP32S3" width="120"></td>
      <td><strong>reSpeaker Flex XVF3800 Circular-4 with XIAO ESP32S3</strong><br><sub>AI Mic Array for Robotics and Embodied AI</sub></td>
      <td><a href="https://www.seeedstudio.com/reSpeaker-Flex-XVF3800-Circular-4-with-XIAO-ESP32S3-p-6739.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/1/-/1-respeaker-xvf3800-4-mic-array.jpg" alt="reSpeaker XMOS XVF3800" width="120"></td>
      <td><strong>reSpeaker XMOS XVF3800</strong><br><sub>AI-powered 4-Mic Array for Clear Voice Even in Noise</sub></td>
      <td><a href="https://www.seeedstudio.com/ReSpeaker-XVF3800-USB-Mic-Array-p-6488.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td rowspan="2" align="center"><strong>Controlador</strong></td>
      <td rowspan="2" align="center">Controlador embebido</td>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/1/1/110110147.jpg" alt="reComputer J3011-Orin Nano 8GB" width="120"></td>
      <td><strong>reComputer J3011-Orin Nano 8GB</strong></td>
      <td><a href="https://www.seeedstudio.com/reComputer-J3011-p-5590.html">Seeed Studio</a></td>
    </tr>
    <tr>
      <td align="center"><img src="https://media-cdn.seeedstudio.com/media/catalog/product/cache/7f7f32ef807b8c2c2215b49801c56084/i/m/image-kit-3.png" alt="NVIDIA Jetson AGX Thor Developer Kit" width="120"></td>
      <td><strong>NVIDIA Jetson AGX Thor Developer Kit</strong></td>
      <td><a href="https://www.seeedstudio.com/NVIDIA-Jetson-AGX-Thor-Developer-Kit-p-9965.html">Seeed Studio</a></td>
    </tr>
  </tbody>
</table>

-----


### 🎓 Ecosistema completo de robótica

reBot-DevArm no es solo un brazo robótico, sino también una comunidad de aprendizaje en robótica. Compartimos los siguientes tutoriales generales de forma gratuita:

#### 🖥️ Computación en el borde y control maestro

* [![Jetson](https://img.shields.io/badge/NVIDIA-reComputer%20Jetson-76B900?style=for-the-badge&logo=nvidia&logoColor=white)](https://wiki.seeedstudio.com/NVIDIA_Jetson/) —— **Inferencia de IA y núcleo de cómputo**
* [![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-4B%20%2F%205-C51A4A?style=for-the-badge&logo=Raspberry%20Pi&logoColor=white)](https://wiki.seeedstudio.com/raspberry-pi-devices/) —— **Entorno de desarrollo Linux general**
* [![ESP32](https://img.shields.io/badge/MCU-Seeed%20XIAO%20(ESP32)-0091BD?style=for-the-badge&logo=espressif&logoColor=white)](https://wiki.seeedstudio.com/SeeedStudio_XIAO_Series_Introduction/) —— **Nodo de control inalámbrico de bajo consumo**

#### 📡 Sensores y periféricos

* **🚗 Motores y servos**: [Damiao / Gogo / Robstride / Mita / Feite / Fashion Star](https://wiki.seeedstudio.com/robotics_page/)
* **👁️ Percepción visual**: [Cámaras de profundidad / LiDAR / Algoritmos de visión](https://wiki.seeedstudio.com/robotics_page/)
* **👂 Interacción auditiva**: [reSpeaker Mic Arrays/Voice Control/Spatial Awareness(DoA)](https://wiki.seeedstudio.com/control_rebot_arm_using_voice_with_respeaker_flex/)
* **🧭 Movimiento y orientación**: [IMU (6 ejes/9 ejes) / Giroscopios / Magnetómetros](https://wiki.seeedstudio.com/Sensor/IMU/)
* **🤖 Kits completos**: [Más sensores y ejemplos de controladores](https://wiki.seeedstudio.com/robotics_page/)

> 👉 **[Haz clic para entrar en la base de conocimiento Wiki](https://wiki.seeedstudio.com/)** (Todos los tutoriales son gratuitos)

---

## 🙌 Referencias y agradecimientos

El camino del código abierto nunca es solitario. El nacimiento del proyecto reBot-DevArm no sería posible sin el apoyo total de Seeed Studio, la comunidad global de código abierto y excelentes socios de hardware. Expresamos nuestro mayor respeto a los siguientes proyectos y equipos:

### 🌍 Ecosistema y soporte de software

* **[Seeed Studio](https://www.seeedstudio.com/)** - Proporciona soporte integral de cadena de suministro y técnico.
* **[Hugging Face LeRobot](https://github.com/huggingface/lerobot)** - Excelente framework de aprendizaje robótico de extremo a extremo.
* **[NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim)** - Potente plataforma de simulación robótica y generación de datos.

### ⚙️ Socios principales de hardware

Gracias a los siguientes fabricantes por proporcionar soluciones de motores y actuadores de alto rendimiento:

* **[Damiao Technology](https://www.damiaokeji.com/)**
* **[Robstride](https://robstride.com/)**
* **[Fashion Star](https://fashionstar.com.hk/wiki/)**

### 💡 Inspiración

Este proyecto está profundamente inspirado en los siguientes proyectos de código abierto:

* **[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/main)**
* **[Mobile ALOHA](https://github.com/tonyzhaozh/aloha)**
* **[Dummy-Robot (Zhihui Jun)](https://github.com/peng-zhihui/Dummy-Robot)**
* **[OpenArm](https://openarm.dev/)**
* **[I2RT](https://i2rt.com/)**
* **[TRLC-DK1](https://github.com/robot-learning-co/trlc-dk1)**

### 🎃 Contribuidores del prototipo

* **Equipo de robótica AI de SeeedStudio**: Yaohui Zhu (yaohui.zhu@seeed.cc)
* **SeeedStudio STU**: Wentao Dong
* **SeeedStudio STU**: Weiwei Xu
* **Departamento de compras de SeeedStudio**: Fengqun Peng

### 👥 Contributors

## Our Top Contributors

<p align="center"><a href="https://github.com/Seeed-Projects/reBot-DevArm/graphs/contributors">
  <img src="https://contributors-img.web.app/image?repo=Seeed-Projects/reBot-DevArm" />
</a></p>

*Coming soon... Welcome to submit PRs to become a contributor!*

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=Seeed-Projects/reBot-DevArm&type=date&legend=top-left)](https://www.star-history.com/#Seeed-Projects/reBot-DevArm&type=date&legend=top-left)

# Licencia del proyecto reBot-DevArm

- **Diseño de hardware** © 2026 Seeed Studio Co., Ltd. (SeeedStudio), publicado bajo licencia [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt)
- **Código del firmware** © 2026 Seeed Studio Co., Ltd. (SeeedStudio), publicado bajo licencia [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)

## Derechos y restricciones

Estimados desarrolladores y expertos de la industria, el proyecto del brazo robótico reBot Arm siempre se ha adherido a los valores fundamentales de **Agilidad, Apertura, Responsabilidad y Simbiosis** para servir a la comunidad de desarrolladores. Nuestra visión es permitir que cada entusiasta domine sistemáticamente la arquitectura del hardware y los principios del software de los brazos robóticos, y experimente de forma inmersiva los algoritmos de vanguardia de la inteligencia corporeizada a través del proyecto reBot.

Durante los primeros cinco meses desde su lanzamiento, el proyecto ha utilizado la licencia de código abierto **CC BY-SA NC (No Comercial)**. La intención original era permitir que todos los desarrolladores y contribuyentes se concentraran en iterar y mejorar el producto durante su fase inicial, menos madura, sin ser molestados por preocupaciones comerciales, y dedicarse plenamente a la co-construcción y optimización del proyecto.

Después de meses de profundo pulido del producto y maduración técnica por parte de Seeed Studio, **a partir del 11 de mayo de 2026**, el proyecto reBot Arm ha pasado oficialmente de la licencia CC BY-SA NC a la licencia de código abierto **CERN-OHL-W 2.0**.

A partir de este momento, el proyecto logra un **código abierto del 100% en toda la cadena (hardware y software)** , otorgando **derechos de uso comercial completos para todos los escenarios**.

Esperamos que continúen participando con un espíritu inclusivo y colaborativo, para sostener, mantener y profundizar la comunidad de código abierto reBot Arm, compartir los frutos del código abierto y construir juntos un ecosistema para la inteligencia corporeizada.

Este proyecto utiliza diferentes licencias de código abierto para Hardware y Software. Por favor, confirme los términos de la licencia aplicables a la parte que está utilizando.

| Elemento / Licencia                    | Hardware de reBot: CERN-OHL-W-2.0                              | SDK de software de reBot: Apache-2.0                         |
| -------------------------------------- | -------------------------------------------------------------- | ------------------------------------------------------------ |
| **✅ Uso comercial permitido**         | ✅ Permitido                                                   | ✅ Permitido                                                 |
| **✅ Modificación permitida**          | ✅ Permitido                                                   | ✅ Permitido                                                 |
| **✅ Redistribución permitida**        | ✅ Permitido                                                   | ✅ Permitido                                                 |
| **✅ Integración/redistribución en código cerrado** | ❌ Condicional (ver [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt) para más detalles) | ✅ Permitido (no es necesario divulgar el código modificado) |
| **⚠️ Obligación de conservar el aviso de copyright** | ✅ Obligatorio                                                 | ✅ Obligatorio                                               |
| **⚠️ Obligación de conservar el texto de la licencia** | ✅ Obligatorio                                                 | ✅ Obligatorio                                               |
| **⚠️ Notificación de modificaciones requerida** | ✅ Obligatorio (con fecha y descripción)                      | ✅ Obligatorio (con descripción de las modificaciones)       |
| **⚠️ Concesión de patentes**           | ✅ Concesión explícita de patentes (ver [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt) para más detalles) | ✅ Concesión explícita de patentes                           |
| **⚠️ Obligación de proporcionar las fuentes al distribuir** | ✅ **Obligatorio** proporcionar las "Fuentes Completas" del hardware | ❌ Sin obligación de proporcionar las fuentes                |
| **⚠️ Compatibilidad con módulos externos/cerrados** | ✅ Permitido (característica Weakly Reciprocal)               | ✅ Totalmente permitido                                      |
| **🔗 Relación con otros componentes/módulos** | Los módulos de interfaz independientes (External Material) pueden conservar su licencia original (cerrada) | Sin restricciones, puede enlazarse con bibliotecas bajo cualquier licencia |
| **📄 Texto oficial de la licencia**    | [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2.txt)          | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)    |

## ☎ Contact Us

* **Progreso de código abierto y soporte técnico**-Yaohui: yaohui.zhu@seeed.cc
* **Colaboración futura y personalización**-Elaine: elaine.wu@seeed.cc
