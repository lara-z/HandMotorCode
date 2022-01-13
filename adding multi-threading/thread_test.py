import threading
import time
import queue

def check_thread_stop(q):
	if q.empty() == False:
		if q.get() == True:
			stop = True
			return break
	return pass

def start_thread():
	def print_num(num, word):
		count = 0
		stop = False
		while count <= num:
			if q.empty() == False:
				if q.get() == True:
					stop = True
			if stop:
				print('broken')
				break
			print('Combo: ' + str(count) + '. ' + word)
			q.put(word+str(count))
			time.sleep(0.5)
			count += 1
		print('done')
		
	def question():
		print("does this interrupt?")
	
	q = queue.LifoQueue()
	# q.LifoQueue()

	x = threading.Thread(target=print_num, args=(10, "lamb"))
	# y = threading.Thread(target=question)
	x.start()
	
	return x, q

def end_thread(x, q):
	q.put(True)
	# y.start()
	x.join()
	print('thread ended')

x,q = start_thread()
time.sleep(2)
# y.start()
# x.join()
end_thread(x,q)
# y.join()
print(q.get())
print(f'Active Threads: {threading.active_count()}')
