#!/usr/bin/python3

from typing import Callable, List,Dict, Tuple, TypedDict
from skin_real.scn.core import intx_to_int, mask,mask_to_indices

from skin_real.scn.sc.pkt.data import SENS_NAME_VAL_IND_MAP

import skin_real.scn.sc.pkt.data_6b16 as data_6b16
import skin_real.scn.sc.pkt.data as data_pkt
from skin_real.scn.sc.pkt.tools import get_id

# Event IDs for Event1200

EVENT_ID_PROX   = 1001
EVENT_ID_FORCE1 = 1002
EVENT_ID_FORCE2 = 1003
EVENT_ID_FORCE3 = 1004
EVENT_ID_ACCX   = 1005
EVENT_ID_ACCY   = 1006
EVENT_ID_ACCZ   = 1007
EVENT_ID_TEMP   = 1008


SENS_NAME_EVENT_ID_MAP : Dict[str,int] = {
    "prox"      : EVENT_ID_PROX,
    "force1"    : EVENT_ID_FORCE1,
    "force2"    : EVENT_ID_FORCE2,
    "force3"    : EVENT_ID_FORCE3,
    "acc_x"     : EVENT_ID_ACCX,
    "acc_y"     : EVENT_ID_ACCY,
    "acc_z"     : EVENT_ID_ACCZ,
    "temp"      : EVENT_ID_TEMP,
}

EVENT_ID_SENS_NAME_MAP : Dict[int,str] = { v : k for k,v in SENS_NAME_EVENT_ID_MAP.items() }
EVENT_ID_SENS_IND_MAP : Dict[int,int] = { v : SENS_NAME_VAL_IND_MAP[k] for k,v in SENS_NAME_EVENT_ID_MAP.items() }

SENS_IND_EVENT_ID_MAP : Dict[int,int] = { v : k for k,v in EVENT_ID_SENS_IND_MAP.items() }


def event_index_to_id(sens_ind : int) -> int:
    return SENS_IND_EVENT_ID_MAP[sens_ind]


EventTuple = Tuple[int,int,float]       # tuple: sc_id, id, value
EventTupleList = List[EventTuple]

class Event(TypedDict):
    sc_id:          int                      # skin cell ID
    id:             int                      # event ID (sensor ID)
    value:          float                    # sensor value  

EventList = List[Event]


def tuple_to_event(et : EventTuple) -> Event:
    return Event(sc_id=et[0],id=et[1],value=et[2])

def event_to_tuple(e : Event) -> EventTuple:
    return (e["sc_id"], e["id"], e["value"])

def tuples_to_events(etl : EventTupleList) -> EventList : 
    return [ tuple_to_event(et) for et in etl ]


def events_to_tuples(es : EventList) -> EventTupleList : 
    return [ event_to_tuple(e) for e in es ]


def get_pkt_ind(pkt : bytes) -> int:
    m4 = mask(4)
    v = 0
    v |= (pkt[3] >> 2) & m4     # 4 bits
    return v


def get_active_events_mask(pkt : bytes) -> int:
    m2 = mask(2)
    m7 = mask(7)
    v = 0
    v |= (pkt[3] << 7) & (m2 << 7)  # 2 bits
    v |= pkt[4] & m7                # 7 bits
    return v


def get_event_indices(pkt : bytes) -> List[int]:
    m = get_active_events_mask(pkt)
    return mask_to_indices(m,8)


def get_event_ids(pkt : bytes) -> List[int]:
    inds = get_event_indices(pkt)
    ids = []
    for ind in inds:
        ids.append(SENS_IND_EVENT_ID_MAP[ind])
    return ids


def get_raw_event_value(pkt : bytes, ind : int) -> int:
    return data_6b16.get_value(pkt,ind)


def data_conv_to_prox(raw : int) -> float:
    return data_pkt.data_conv_to_prox(raw)

def data_conv_to_force(raw : int) -> float:
    return data_pkt.data_conv_to_force(raw)


def data_conv_to_acc(raw : int) -> float:
    return data_pkt.data_conv_to_acc(intx_to_int(raw,10))

def data_conv_to_temp(raw : int) -> float:
    return data_pkt.data_conv_to_temp(intx_to_int(raw,8))


CONV_RAW_VALUE_FUNCS : List[Callable[[int],float]] = [
        data_conv_to_prox,
        data_conv_to_force,
        data_conv_to_force,
        data_conv_to_force,
        data_conv_to_acc,
        data_conv_to_acc,
        data_conv_to_acc,
        data_conv_to_temp
    ]


def get_event_value(pkt : bytes, value_ind : int, event_ind : int):
    v_raw = get_raw_event_value(pkt,value_ind)
    return CONV_RAW_VALUE_FUNCS[event_ind](v_raw)


def get_event_tuples(pkt : bytes) -> EventTupleList :
    e_inds = get_event_indices(pkt) # 
    pkt_ind = get_pkt_ind(pkt)

    n_events = len(e_inds)

    # default: less than 6 events
    ind_s = 0
    ind_e = n_events

    # second packet, skip first 6 events
    if n_events > 6 and pkt_ind != 0:
        ind_s = 6
        ind_e = n_events

    # first packet
    if n_events > 6 and pkt_ind == 0:
        ind_s = 0
        ind_e = 6

    sc_id = get_id(pkt)

    etl : EventTupleList = []

    for ind in range(ind_s,ind_e):
        e_ind = e_inds[ind]
        v_ind = ind - ind_s
        e_val = get_event_value(pkt,v_ind,e_ind)

        # print(f"e_ind = {e_ind}")

        # fix acc mapping errors: accX -> accY
        if e_ind == data_pkt.SENS_IND_ACCX:
            e_id = event_index_to_id(data_pkt.SENS_IND_ACCY)
            et = (sc_id,e_id,e_val)
            etl.append( et )
            continue

        # fix acc mapping errors: accY -> -accX
        if e_ind == data_pkt.SENS_IND_ACCY:
            e_id = event_index_to_id(data_pkt.SENS_IND_ACCX)
            et = (sc_id,e_id,-e_val)
            etl.append( et )
            continue
        
        e_id = event_index_to_id(e_ind)
        et = (sc_id,e_id,e_val)
        etl.append( et )

    return etl

def get_events(pkt : bytes) -> EventList :
    etl = get_event_tuples(pkt)
    return tuples_to_events(etl)