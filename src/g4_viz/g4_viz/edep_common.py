#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from typing import Optional

# g4_interfaces.msg.Edep があればそちらを優先、無ければ Float32 で受ける
try:
    from g4_interfaces.msg import Edep as EdepMsg
except Exception:
    EdepMsg = None
from std_msgs.msg import Float32

def edep_msg_types():
    """購読を試みる型（片方しか接続されない想定）"""
    types = []
    if EdepMsg is not None:
        types.append(EdepMsg)
    types.append(Float32)
    return types

def extract_edep_value(msg) -> Optional[float]:
    """メッセージからエネルギー値（MeV）を抽出"""
    # g4_interfaces/Edep に想定される候補フィールド
    for key in ('edep', 'energy', 'energy_mev', 'value', 'data'):
        if hasattr(msg, key):
            v = getattr(msg, key)
            try:
                return float(v)
            except Exception:
                pass
    return None
