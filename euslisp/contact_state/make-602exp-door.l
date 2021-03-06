(defclass 602exp-door
  :super cascaded-link
  :slots (rear-knob-coords front-knob-coords end-coords l1 l2 l3 j1 j2 door-radius door-width door-thickness knob-height knob-radius front-touch-cds-list rear-touch-cds-list h-tgts-num v-tgts-num h-step v-step))

(defmethod 602exp-door
  (:init (door-radius knob-height knob-radius)
	 (let (b)
           (setq door-width (+ door-radius (+ knob-radius 50)))
           (setq door-thickness 30)
	   (send-super :init)
	   (setq b (make-cube 30 (* 2 knob-radius) 15))
	   (send b :locate (float-vector 0 knob-radius 0))
	   (send b :set-color :red)
	   (setq l3 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'front-knob))
           (setq front-knob-coords (make-cascoords :coords (send (send l3 :copy-worldcoords) :translate 
                                                                 (float-vector 0 knob-radius 0))
                                                   :name "front-knob-coords"))
           (send l3 :assoc front-knob-coords)
	   ;; (send l3 :assoc end-coords)
	   ;; (send l3 :locate (float-vector (+ 25 (- (+ 50 (/ 25 2.0))))  (* -1 door-radius) knob-height))
	   (send l3 :locate (float-vector (* -1 (+ 50 (/ door-thickness 2)))  (* -1 door-radius) knob-height))
           
           
           (setq b (make-cube 30 (* 2 knob-radius) 15))
	   (send b :locate (float-vector 0 knob-radius 0))
           (send b :set-color :red)
           (setq l4 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'rear-knob))
           ;; (send l4 :locate (float-vector (+ 25 125 (- (+ 50 (/ 25 2.0))))  (* -1 door-radius) knob-height))
           (send l4 :locate (float-vector (+ 50 (/ door-thickness 2)) (* -1 door-radius) knob-height))
           (setq rear-knob-coords (make-cascoords :coords (send (send l4 :copy-worldcoords) :translate
                                                                (float-vector 0 knob-radius 0))
                                                  :name "rear-knob-coords"))
           (send l4 :assoc rear-knob-coords)
           (send l3 :assoc l4)

	   
	   (setq b (make-cube door-thickness door-width (* 2 knob-height)))
	   ;; (send b :locate (float-vector 25 (/ (+ door-radius 95) -2.0) knob-height))
	   (send b :locate (float-vector (/ door-thickness 2) (/ door-width -2.0) knob-height)) ;;shift for hinge axis
           (send l3 :translate (float-vector (/ door-thickness 2) 0 0)) ;;shift-for hinge axis
	   (send b :set-color :yellow 0.8)
	   (setq l2 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-plate))
	   (send l2 :assoc l3)
	   (send l2 :translate (float-vector (/ door-thickness 2) (/ door-thickness -2) 0))
           
           (setq b (make-cube door-thickness door-width (* 2 knob-height)))
           (send b :locate (float-vector (/ door-thickness 2) (* 3 (/ door-width -2.0)) knob-height))
	   (send b :set-color :grey 0.8)
	   (setq l5 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'wall))
	   (send l5 :translate (float-vector (/ door-thickness 2) (/ door-thickness -2) 0))
	   
	   (setq b (make-cube 10 10 (* 2 knob-height)))
	   (send b :locate (float-vector (/ door-thickness 2) 0 knob-height))
	   (send b :set-color :grey)
	   (setq l1 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-hinge))
	   (send l1 :assoc l2)
           (send l1 :assoc l5)
           (send self :assoc l1)
	   
	   (setq j1 (instance rotational-joint :init :name 'j1
			      :parent-link l1 :child-link l2 :axis :z :min -100 :max 100))
	   (setq j2 (instance rotational-joint :init :name 'j1
			      :parent-link l2 :child-link l3 :axis :x :min -60 :max 0))
	   
	   (setq links (list l1 l2 l3 l4 l5))
	   (setq joint-list (list j1 j2))

           (setq h-tgts-num 4)
           (setq v-tgts-num 6)
           (send self :make-front-touch-cds-list door-width knob-height h-tgts-num v-tgts-num)
           (send self :make-rear-touch-cds-list door-width knob-height h-tgts-num v-tgts-num)
	   (send self :init-ending)
	   self))

  (:h-tgts-num
   ()
   h-tgts-num)

  (:v-tgts-num
   ()
   v-tgts-num)


  (:make-front-touch-cds-list
      (door-width knob-height &optional (p 8) (q 8))
   (let (cds-list
         cds)
     ;; (y-step (/ (/ door-width 2) p))
     ;; (z-step (/ (/ knob-height 2) q)))
     (setq h-step (/ (/ door-width 1) p))
     (setq h-offset 50) ;;avoid too close to door edge
     (setq v-step  (/ (/ door-width 0.8) q))
;;     (setq v-step (/ (/ knob-height 1) q))
     (dotimes (i p)  
       (dotimes (j q)
         (setq cds (make-cascoords :coords (send l2 :copy-worldcoords)))
         (send cds :translate (float-vector -1 (- door-width) (* 1.4 knob-height)))
         (send cds :translate (float-vector 0 (+ (* i h-step) h-offset) (- (* j v-step))))
         (send (elt (send self :links) 1) :assoc cds) ;;assoc to door-plane link
         (push cds cds-list)))
     (setq front-touch-cds-list cds-list)
     (setq front-touch-cds-list (reverse front-touch-cds-list))
     cds-list))


  (:make-rear-touch-cds-list
   (door-width knob-height &optional (p 8) (q 8))
   (let (cds-list
         cds)
     ;; (y-step (/ (/ door-width 2) p))
     ;; (z-step (/ (/ knob-height 2) q)))
     (setq h-step (/ (/ door-width 1) p))
     (setq h-offset 50) ;;avoid too close to door edge
     (setq v-step  (/ (/ door-width 0.8) q))
;;     (setq v-step (/ (/ knob-height 1) q))
     (dotimes (i p)  
       (dotimes (j q)
         (setq cds (make-cascoords :coords (send l2 :copy-worldcoords)))
         (send cds :translate (float-vector (+ 1 door-thickness) (- door-width) (* 1.4 knob-height)))
         (send cds :translate (float-vector 0 (+ (* i h-step) h-offset) (- (* j v-step))))
         (send (elt (send self :links) 1) :assoc cds) ;;assoc to door-plane link
         (push cds cds-list)))
     (setq rear-touch-cds-list cds-list)
     (setq rear-touch-cds-list (reverse rear-touch-cds-list))
     cds-list))


  (:get-front-touch-cds-list
   ()
   front-touch-cds-list)
  
  (:get-rear-touch-cds-list
   ()
   rear-touch-cds-list)

  (:rear-knob-coords
   ()
   rear-knob-coords)

  (:front-knob-coords
   ()
   front-knob-coords)
  
  
  (:end-coords 
   (&rest args) 
   (forward-message-to end-coords args))

  (:knob 
   (&rest args) 
   (forward-message-to (cadr (send self :joint-list)) args))
  (:hinge 
   (&rest args) 
   (forward-message-to (car (send self :joint-list)) args))
  )




(defun make-602exp-door ()
  ;; (setq *door* (instance door :init 700 1000 50))
  ;; (send *door* :translate #f(700 700 0))
  (let (d)
  (setq d (instance 602exp-door :init 646 920 (/ 100 2)))
  (send d :translate #f(700 700 0))
  ;; (send *irtviewer* :objects *door*)
  ;; (send *irtviewer* :objects (append ( send *irtviewer* :objects) (send *door* :get-rear-touch-cds-list)))
  d))
