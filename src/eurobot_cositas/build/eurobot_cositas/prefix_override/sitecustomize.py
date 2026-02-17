import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wormi/sprint5/camara/ASDASDA/g06_prii3_wsv2/src/eurobot_cositas/install/eurobot_cositas'
