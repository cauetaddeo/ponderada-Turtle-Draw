import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cauex/workspace_visao_comp/src/ponderada_vc/install/ponderada_vc'
