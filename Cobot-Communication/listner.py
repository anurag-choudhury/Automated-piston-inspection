
import asyncio
import techmanpy
import keyboard
import camera_code as cc

async def isListenActive():
   async with techmanpy.connect_sta(robot_ip='192.168.98.99') as conn:
        print("in listen active ")
        print(conn.is_listen_node_active())
        return await conn.is_listen_node_active()
    
asyncio.run(isListenActive())
