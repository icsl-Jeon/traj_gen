## 2020/3/20
- error handling when unsolved (solve fun / eval fun)
- Implemented end-derivative method. For better accuracy, better to use "double" , not "float" due to the inversion 
- Templating all the trajectory generation process. 
(observed that the gap w.r.t accuracy and speed of *float* and *double* was considerable)
-  The doxygen file is loaded. Check *cpp/docs/html/index.html*.
- OptimTrajGen was fully implemented. 