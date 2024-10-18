#!/usr/bin/env python

import asyncio
import techmanpy
from techmanpy import TechmanException

async def main():
   try: 
        async with techmanpy.connect_sct(robot_ip='192.168.98.99') as conn:
         # connection established
            await conn.move_to_joint_angles_ptp([10, -10, 10, -10, 10, 10], 0.80, 200)

            # await conn.resume_project()
            print("conn established")
            print(conn)
   except TechmanException as e: print(e) # connection could not be established

try: asyncio.run(main())
except KeyboardInterrupt: pass # user interrupted with Ctrl+C