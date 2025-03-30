import gc
import machine

gc.collect()

mem_allocated = gc.mem_alloc()
mem_free = gc.mem_free()

print(f"Mémoire allouée : {mem_allocated} bytes")
print(f"Mémoire libre : {mem_free} bytes")
