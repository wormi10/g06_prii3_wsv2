#!/usr/bin/env python3
"""
Script de diagnóstico para verificar la configuración del detector de despensas
"""

import os
import sys

def check_installation():
    print("="*70)
    print("🔍 DIAGNÓSTICO DEL DETECTOR DE DESPENSAS")
    print("="*70)
    
    # Verificar directorio actual
    print(f"\n📁 Directorio actual: {os.getcwd()}")
    
    # Buscar el paquete
    ros_workspace = None
    for path in ['/home/arsalpal/v7', '/home/arsalpal/ros2_ws', os.path.expanduser('~/v7')]:
        if os.path.exists(path):
            ros_workspace = path
            break
    
    if ros_workspace:
        print(f"✓ Workspace encontrado: {ros_workspace}")
        
        pkg_path = os.path.join(ros_workspace, 'src', 'eurobot_cositas')
        if os.path.exists(pkg_path):
            print(f"✓ Paquete eurobot_cositas encontrado: {pkg_path}")
            
            # Verificar carpeta templates
            templates_path = os.path.join(pkg_path, 'templates')
            print(f"\n📂 Carpeta templates:")
            if os.path.exists(templates_path):
                print(f"  ✓ Existe: {templates_path}")
                files = os.listdir(templates_path)
                if files:
                    print(f"  ✓ Archivos encontrados:")
                    for f in files:
                        print(f"    - {f}")
                else:
                    print("  ❌ Carpeta vacía")
            else:
                print(f"  ❌ NO existe: {templates_path}")
                print(f"  💡 Necesitas crearla: mkdir -p {templates_path}")
            
            # Verificar scripts
            print(f"\n📜 Scripts:")
            scripts_path = os.path.join(pkg_path, 'scripts')
            if os.path.exists(scripts_path):
                detector_path = os.path.join(scripts_path, 'aruco_detector_cositas.py')
                if os.path.exists(detector_path):
                    print(f"  ✓ Detector encontrado: {detector_path}")
                    
                    # Verificar si tiene el código de detección de despensas
                    with open(detector_path, 'r') as f:
                        content = f.read()
                        if 'enable_storage_detection' in content:
                            print(f"  ✓ Código de detección de despensas PRESENTE")
                        else:
                            print(f"  ❌ Código de detección de despensas NO ENCONTRADO")
                            print(f"  💡 Necesitas reemplazar el archivo con aruco_detector_with_storage.py")
                else:
                    print(f"  ❌ Detector NO encontrado: {detector_path}")
            
            # Verificar launch
            print(f"\n🚀 Launch files:")
            launch_path = os.path.join(pkg_path, 'launch')
            if os.path.exists(launch_path):
                for launch_file in ['Move_eurobot_launch.py', 'Move_eurobot_launch_UPDATED.py']:
                    launch_full = os.path.join(launch_path, launch_file)
                    if os.path.exists(launch_full):
                        with open(launch_full, 'r') as f:
                            content = f.read()
                            if 'enable_storage_detection' in content:
                                print(f"  ✓ {launch_file} - CON parámetros de despensas")
                            else:
                                print(f"  ⚠️  {launch_file} - SIN parámetros de despensas")
        else:
            print(f"❌ Paquete NO encontrado: {pkg_path}")
    else:
        print("❌ Workspace NO encontrado")
    
    print("\n" + "="*70)
    print("📋 RESUMEN DE ACCIONES NECESARIAS:")
    print("="*70)
    
    # Generar lista de acciones
    actions = []
    
    if ros_workspace and os.path.exists(os.path.join(ros_workspace, 'src', 'eurobot_cositas')):
        pkg_path = os.path.join(ros_workspace, 'src', 'eurobot_cositas')
        templates_path = os.path.join(pkg_path, 'templates')
        
        if not os.path.exists(templates_path):
            actions.append(f"mkdir -p {templates_path}")
            actions.append(f"cp storage_template.png {templates_path}/")
        elif not os.listdir(templates_path):
            actions.append(f"cp storage_template.png {templates_path}/")
        
        detector_path = os.path.join(pkg_path, 'scripts', 'aruco_detector_cositas.py')
        if os.path.exists(detector_path):
            with open(detector_path, 'r') as f:
                if 'enable_storage_detection' not in f.read():
                    actions.append(f"cp aruco_detector_with_storage.py {detector_path}")
        
        launch_updated = os.path.join(pkg_path, 'launch', 'Move_eurobot_launch_UPDATED.py')
        launch_original = os.path.join(pkg_path, 'launch', 'Move_eurobot_launch.py')
        if not os.path.exists(launch_updated):
            actions.append(f"cp Move_eurobot_launch_UPDATED.py {pkg_path}/launch/")
    
    if actions:
        print("\nEjecuta estos comandos:\n")
        for i, action in enumerate(actions, 1):
            print(f"{i}. {action}")
        
        print("\nDespués:")
        print(f"cd {ros_workspace}")
        print("colcon build --packages-select eurobot_cositas")
        print("source install/setup.bash")
    else:
        print("\n✅ Todo parece estar en orden!")
        print("\n💡 Si aún no detecta despensas, verifica:")
        print("   1. Que Gazebo esté mostrando el tablero con las despensas verdes")
        print("   2. En RViz2 abre el topic /aruco/detected_image")
        print("   3. Revisa los logs en busca de errores de carga del template")

if __name__ == "__main__":
    check_installation()
