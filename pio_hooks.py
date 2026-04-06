Import("env")
import subprocess
import os

def run_collect_code(source, target, env):
    print("\n[PRE-ACTION] Running collect_code.py to update snapshot...")
    # Путь к вашему скрипту
    script_path = os.path.join(env.get("PROJECT_DIR"), "collect_code.py")
    
    # Запуск скрипта через интерпретатор Python
    try:
        subprocess.run(["python", script_path], check=True)
        print("[PRE-ACTION] Snapshot updated successfully!\n")
    except Exception as e:
        print(f"[PRE-ACTION] Error running collect_code.py: {e}\n")

# Привязываем запуск к процессу загрузки (Upload)
# Если хотите запускать и при сборке (Build), используйте "buildprog"
env.AddPreAction("upload", run_collect_code)