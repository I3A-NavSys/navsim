import pygame
import asyncio

class KeyboardInput:
    def __init__(self):
        self.inputs = [0,0,0,0]
        self.event = asyncio.Event()
        self.acc = 0.03
        

    def start(self):
        # Loop condition
        self._stop = False

        # Initialize pygame environment
        if not pygame.get_init():
            print("NOT INITIALIZED")
            pygame.init()
        else:
            print("INITIALIZED")
        
        # Start input coroutine
        asyncio.ensure_future(self.input())


    def stop(self):
        self._stop = True
        self.event.set()

        # Finish pygame environment
        pygame.quit()

    
    def ask_input(self):
        self.event.set()


    async def input(self):
        while not self._stop:
            await self.event.wait()

            # Avoid exception when saving file while running simulation
            if self._stop:
                break
            
            for event in pygame.event.get():
                print("HERE")
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        self.inputs[0] = -0.1
                        print("P")

                    if event.key == pygame.K_s:
                        self.inputs[0] = 0.1
                        print("P")

                    if event.key == pygame.K_a:
                        self.inputs[1] = -0.1
                        print("P")

                    if event.key == pygame.K_d:
                        self.inputs[1] = 0.1
                        print("P")

                    if event.key == pygame.K_UP:
                        self.inputs[2] = -0.1
                        print("P")

                    if event.key == pygame.K_DOWN:
                        self.inputs[2] = 0.1
                        print("P")

                    if event.key == pygame.K_LEFT:
                        self.inputs[3] = -0.1
                        print("P")

                    if event.key == pygame.K_RIGHT:
                        self.inputs[3] = 0.1
                        print("P")

                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_w:
                        self.inputs[0] = 0
                        print("R")

                    if event.key == pygame.K_s:
                        self.inputs[0] = 0
                        print("R")

                    if event.key == pygame.K_a:
                        self.inputs[1] = 0
                        print("R")

                    if event.key == pygame.K_d:
                        self.inputs[1] = 0
                        print("R")

                    if event.key == pygame.K_UP:
                        self.inputs[2] = 0
                        print("R")

                    if event.key == pygame.K_DOWN:
                        self.inputs[2] = 0
                        print("R")

                    if event.key == pygame.K_LEFT:
                        self.inputs[3] = 0
                        print("R")

                    if event.key == pygame.K_RIGHT:
                        self.inputs[3] = 0
                        print("R")

            self.event.clear()