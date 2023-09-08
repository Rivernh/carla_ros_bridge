#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import yaml
import numpy as np
import cv2
from PIL import Image

class Config:
	def __init__(self, cfg):
		self.cfg = cfg

	def __getattr__(self, key):
		if isinstance(self.cfg[key], dict):
			return Config(self.cfg[key])
		
		return self.cfg[key]

def load_yaml(file_path) -> dict:
	with open(file_path, 'r', encoding="utf-8") as file:
		result = yaml.unsafe_load(file)
	return result

def load_config(file_path):
	_cfg = load_yaml(file_path)
	return Config(_cfg)

class Singleton(object):
    _instance = None

    def __new__(cls, *args, **kw):
        if not cls._instance:
            cls._instance = super(Singleton, cls).__new__(cls)  
        return cls._instance
    
class GlobalDict(Singleton):
    def __init__(self, dict={}):
        self.dict = dict
        
    def __getitem__(self, key):
        if key in self.dict:
            return self.dict[key]
        else:
            debug(info='No key called '+ str(key), info_type='error')
            return None
    
    def __setitem__(self, key, value):
        self.dict[key] = value   
        
def debug(info, info_type='debug'):
	if info_type == 'error':
		print('\033[1;31m ERROR:', info, '\033[0m')
	elif info_type == 'success':
		print('\033[1;32m SUCCESS:', info, '\033[0m')
	elif info_type == 'warning':
		print('\033[1;34m WARNING:', info, '\033[0m')
	elif info_type == 'debug':
		print('\033[1;35m DEBUG:', info, '\033[0m')
	else:
		print('\033[1;36m MESSAGE:', info, '\033[0m')
  
def fig2data(fig):
    # draw the renderer
    fig.canvas.draw()
 
    # Get the RGBA buffer from the figure
    w, h = fig.canvas.get_width_height()
    buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf.shape = (w, h, 4)
 
    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = np.roll(buf, 3, axis=2)
    image = Image.frombytes("RGBA", (w, h), buf.tobytes())
    image = np.asarray(image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image

def add_alpha_channel(img): 
    b_channel, g_channel, r_channel = cv2.split(img)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
    alpha_channel[:, :int(b_channel.shape[0] / 2)] = 100
    img_BGRA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
    return img_BGRA