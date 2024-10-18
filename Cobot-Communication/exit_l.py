import asyncio
import techmanpy
import keyboard
import camera_code as cc


async def exitListen():
    async with techmanpy.connect_sct(robot_ip='192.168.98.99') as conn:
        #   rin_listen_node = await conn.is_listen_node_active()
            print("gonna exit listen")
            rin_listen_node = await conn.exit_listen()
            # return rin_listen_node
        
        
asyncio.run(exitListen())
