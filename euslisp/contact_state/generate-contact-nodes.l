(setq debugview-colli nil)
(setq debugview-mintq nil)
(setq debugview-bridge nil)
;; (setq debugview-colli :no-message)
;; (setq debugview-mintq :no-message)



(defun set-contact-points (&optional (arm :larm) &key (robot *pr2*))
  (let (pt0 pt1 pt2 pt3)
    (setq pt0 (make-cascoords :coords (send robot arm :end-coords :copy-worldcoords)))
    (send (send robot arm :end-coords :parent) :assoc pt0)
    
    (setq pt1 (make-cascoords :coords (send (elt (send robot arm :links) 4) :copy-worldcoords))) ;;r_forearm_roll_link
    (send pt1 :translate #f(200 0 -50))
    (send (elt (send robot arm :links) 4) :assoc pt1)
    
    (setq pt2 (make-cascoords :coords (send (elt (send robot arm :links) 3) :copy-worldcoords)))
    (send pt2 :translate #f(-70 0 -70))
    (send (elt (send robot arm :links) 2) :assoc pt2) ;;; previously 3, 2 is supposedly correct
    (list pt0 pt1 pt2)
    ))

(defun init-all-container-nodes ()
  (setq *all-nodes* nil)
  ;;--------------------------start nodes-------------------------------------
  (let ((rsn (instance extended-node :init "r-0-0-fk"))
        (lsn (instance extended-node :init "l-0-0-fk")))

    (dolist (n (list rsn lsn))
      (send n :put-value :arm (if (equal n rsn) :rarm :larm))
      (send n :put-value :contact-point-index 0)
      (send n :put-value :contact-point (elt (set-contact-points (send n :get-value :arm)) 0))
      (send n :put-value :contact-type 'grasp)
      (send n :put-value :graph-depth 0)
      (send n :put-value :door-angle (* 0 (* -1 *door-rot-step*)))
      ;; ;                  (send n :put-value :pose-idx k)
      (send n :put-value :solved nil)
      (send n :put-value :robot-pose nil)
      (send n :put-value :robot-pose-optimized nil)
      (send n :put-value :base-coords nil)
      (send n :put-value :base-coords-optimized nil)
      (send n :put-value :object-coords-index 'fk)
      (send n :put-value :object-coords (send *door* :front-knob-coords))
      (send n :put-value :ik-target-coords (send (send *door* :front-knob-coords) :assoc (make-cascoords :coords (send (send (send (send (send *door* :front-knob-coords) :copy-worldcoords) :rotate  0 :z) :rotate pi/2 :x) :translate #f(15 0 0)))))
      (send n :put-value :torque nil)
      (send n :put-value :torque-optimized nil)
      (send n :put-value :overload? nil)
      (push n *all-nodes*)
      )
    )
  ;;--------------------------start nodes-------------------------------------
  
  (dolist (arm (list :rarm :larm))
    (let ((robot-contact-pts (set-contact-points arm))
          (obj-cds-list (send *door* :get-rear-touch-cds-list))
          (graph-depth *graph-depth*))
      (dotimes (robot-contact-pts-idx (length robot-contact-pts))
        (do ((depth 1 (+ 1 depth))) ((>= depth graph-depth))
;;;-------------------point/line contact nodes-----------------------------;;;
            (dotimes (obj-cds-idx (length obj-cds-list))
              (let ((n (instance extended-node :init (format nil "~A-~A-~A-~A"
                                                             (if (equal arm :rarm) 'r 'l) robot-contact-pts-idx depth obj-cds-idx))))
                
                (send n :put-value :arm arm)
                (send n :put-value :contact-point-index robot-contact-pts-idx)
                (send n :put-value :contact-point (elt robot-contact-pts robot-contact-pts-idx))
                (send n :put-value :contact-type (case robot-contact-pts-idx (1 'line) (t 'point)))
                (send n :put-value :graph-depth depth)
                (send n :put-value :door-angle (* depth (* -1 *door-rot-step*)))
                ;; ;                  (send n :put-value :pose-idx k)
                (send n :put-value :solved nil)
                (send n :put-value :robot-pose nil)
                (send n :put-value :robot-pose-optimized nil)
                (send n :put-value :base-coords nil)
                (send n :put-value :base-coords-optimized nil)
                (send n :put-value :object-coords-index obj-cds-idx)
                (send n :put-value :object-coords (elt obj-cds-list obj-cds-idx))
                (send n :put-value :ik-target-coords (send (elt obj-cds-list obj-cds-idx) :assoc (make-cascoords :coords (send (send (send (elt obj-cds-list obj-cds-idx) :copy-worldcoords) :translate *ik-target-shift*) :rotate (if (eq robot-contact-pts-idx 0) 0 pi/2) :y))))
                (send n :put-value :torque nil)
                (send n :put-value :torque-optimized nil)
                (send n :put-value :overload? nil)

                (push n *all-nodes*)))
;;;-------------------grasp contact nodes-----------------------------;;;
            (dolist (knob-cds (list (send *door* :front-knob-coords) (send *door* :rear-knob-coords)))
              (when (eq robot-contact-pts-idx 0) ;;end-effector
                (let ((n (instance extended-node :init (format nil "~A-~A-~A-~A"
                                                               (if (equal arm :rarm) 'r 'l) robot-contact-pts-idx depth (if (equal (send knob-cds :name) "front-knob-coords") 'fk 'rk)))))
                  
                  (send n :put-value :arm arm)
                  (send n :put-value :contact-point-index robot-contact-pts-idx)
                  (send n :put-value :contact-point (elt robot-contact-pts robot-contact-pts-idx))
                  (send n :put-value :contact-type 'grasp)
                  (send n :put-value :graph-depth depth)
                  (send n :put-value :door-angle (* depth (* -1 *door-rot-step*)))
                  ;; ;                  (send n :put-value :pose-idx k)
                  (send n :put-value :solved nil)
                  (send n :put-value :robot-pose nil)
                  (send n :put-value :robot-pose-optimized nil)
                  (send n :put-value :base-coords nil)
                  (send n :put-value :base-coords-optimized nil)
                  (send n :put-value :object-coords-index (if (equal (send knob-cds :name) "front-knob-coords") 'fk 'rk))
                  (send n :put-value :object-coords knob-cds)
                  (send n :put-value :ik-target-coords (send knob-cds :assoc (make-cascoords :coords (send (send (send (send knob-cds :copy-worldcoords) :rotate (if (equal (send knob-cds :name) "front-knob-coords") 0 pi) :z) :rotate pi/2 :x) :translate #f(15 0 0)))))
                  (send n :put-value :torque nil)
                  (send n :put-value :torque-optimized nil)
                  (send n :put-value :overload? nil)

                  (push n *all-nodes*))))
            ))))
  (setq *all-nodes* (reverse *all-nodes*))
  nil)



(defun init-bridge-nodes (&key (nodes *all-nodes*)) ;;multiple contact
  (setq *bridge-nodes* nil)
  (let (right-candidate-nodes left-candidate-nodes res)
    (dotimes (depth *graph-depth*)
      (setq right-candidate-nodes nil)
      (setq left-candidate-nodes nil)
      
      (dolist (n nodes)
        (if (and (eq (send n :get-value :graph-depth) depth) (send n :get-value :solved) (send n :get-value :robot-pose))
            (if (eq (send n :get-value :arm) :rarm)
                (push n right-candidate-nodes)
              (push n left-candidate-nodes))))

      (send *door* :hinge :joint-angle (* depth  (* -1 *door-rot-step*)))
      (dolist (rn right-candidate-nodes)
        (if (send rn :get-value :robot-pose-optimized)
            (progn (send *pr2* :angle-vector (send rn :get-value :robot-pose-optimized))
                   (send *pr2* :move-to (send rn :get-value :base-coords-optimized) :world))
          (progn (send *pr2* :angle-vector (send rn :get-value :robot-pose))
                 (send *pr2* :move-to (send rn :get-value :base-coords) :world)))
        (dolist (ln left-candidate-nodes)
          (when (not (eq (send rn :get-value :object-coords-index) (send ln :get-value :object-coords-index)))
            (setq res (send *pr2* :inverse-kinematics (list (send rn :get-value :ik-target-coords) (send ln :get-value :ik-target-coords))
                            :move-target (list (send rn :get-value :contact-point) (send ln :get-value :contact-point))
                            :link-list (list (send *pr2* :link-list (send (send rn :get-value :contact-point) :parent)) (send *pr2* :link-list (send (send ln :get-value :contact-point) :parent)))
                            :stop 50
                            :collision-avoidance-link-pair (append (door-arm-base-collision-link-pair (send rn :get-value :contact-point) :use-base t :include-knob (if (eq (send rn :get-value :contact-type) 'grasp) nil t))
                                                                   (door-arm-base-collision-link-pair (send ln :get-value :contact-point) :use-base t :include-knob (if (eq (send ln :get-value :contact-type) 'grasp) nil t)))
                            :avoid-collision-distance 50
                            :avoid-collision-joint-gain 5.0
                            :avoid-collision-null-gain 100.0
                            :rotation-axis (list (case (send rn :get-value :contact-type)
                                                       ('point nil)
                                                       ('line :z)
                                                       ('grasp t)
                                                       (t t))
                                                 (case (send ln :get-value :contact-type)
                                                       ('point nil)
                                                       ('line :z)
                                                       ('grasp t)
                                                       (t t)))
                            :use-torso t
                            :use-base 0.0001 ;;debugged in pr2-util.l
                            :base-range (list :min (float-vector -600 -600 -80)
                                              :max (float-vector 600 600  80))
                            :warnp nil
                            :debug-view debugview-bridge


                            ))
            (when (and res
                     (or
                      (pqp-collision-check-objects ;rarm larm
                       (cdr (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent)))
                       (cdr (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))))
                      (pqp-collision-check-objects ;base+rarm door
                       (append (list (send *pr2* :base_footprint_lk)) (send *pr2* :link-list (send (send rn :get-value :contact-point) :parent)))
                       (case (send rn :get-value :object-coords-index)
                             ('fk (remove-if #'(lambda (x) (substringp "front-knob" (string (send x :name)))) (send *door* :links)))
                             ('rk (remove-if #'(lambda (x) (substringp "rear-knob" (string (send x :name)))) (send *door* :links)))
                             (t (send *door* :links))))
                      (pqp-collision-check-objects ;larm door
                       (send *pr2* :link-list (send (send ln :get-value :contact-point) :parent))
                       (case (send ln :get-value :object-coords-index)
                             ('fk (remove-if #'(lambda (x) (substringp "front-knob" (string (send x :name)))) (send *door* :links)))
                             ('rk (remove-if #'(lambda (x) (substringp "rear-knob" (string (send x :name)))) (send *door* :links)))
                             (t (send *door* :links))))
                      ))
                (setq res nil))

            (when res
              (let ((n (instance extended-node :init (format nil "~A<->~A"
                                                             (send rn :name) (send ln :name)))))
                    
                (send n :put-value :arm :arms)
                (send n :put-value :islands (list (send rn :name) (send ln :name)))
                (send n :put-value :graph-depth depth)
                (send n :put-value :door-angle (* depth (* -1 *door-rot-step*)))
                (send n :put-value :solved t)
                (send n :put-value :robot-pose (send *pr2* :angle-vector))
                (send n :put-value :robot-pose-optimized nil)
                (send n :put-value :base-coords (send *pr2* :copy-worldcoords))
                (send n :put-value :base-coords-optimized nil)
                (send n :put-value :torque nil)
                (send n :put-value :overload? nil)
                
                (push n *bridge-nodes*)))
          )))))
  (setq *bridge-nodes* (reverse *bridge-nodes*))
  )
                          
(defun robot-pose-sampling (&key (robot *pr2*) (door *door*) (nodes *all-nodes*) (obstacle nil) (optimize-exist-pose nil))
  (dolist (n nodes)
    
    (send robot :move-to (make-coords) :world)
    (send robot :reset-pose)
    
    (if optimize-exist-pose
        (progn
          (send robot :angle-vector (send n :get-value :robot-pose))
          (send robot :move-to (send n :get-value :base-coords) :world))
      (progn
        (let (nb-solved 
              upper-n)
          (dolist (nb-n (get-target-neighbour-nodes n :door door)) ;;if neighbour nodes solved, start from that pose to reduce IK solving time
            (if (and (send nb-n :get-value :solved) (send nb-n :get-value :robot-pose))
                (progn
                  (setq nb-solved t)
                  (send robot :angle-vector (send nb-n :get-value :robot-pose))
                  (send robot :move-to (send nb-n :get-value :base-coords) :world)
                  (return))))
          
          (unless nb-solved
            (setq upper-n (get-node (format nil "~A-~A-~A-~A" (if (equal (send n :get-value :arm) :rarm) 'r 'l) (send n :get-value :contact-point-index) (- (send n :get-value :graph-depth) 1) (send n :get-value :object-coords-index))))
            (if (and upper-n (send upper-n :get-value :solved) (send upper-n :get-value :robot-pose)) ;;upper rank node
                (progn 
                  (send robot :angle-vector (send upper-n :get-value :robot-pose))
                  (send robot :move-to (send upper-n :get-value :base-coords) :world)))))
        (send n :put-value :solved t)
        (send n :put-value :robot-pose nil)
        (send n :put-value :robot-pose-optimized nil)
        (send n :put-value :base-coords nil)
        (send n :put-value :base-coords-optimized nil)))
    

    (send door :hinge :joint-angle (send n :get-value :door-angle))

    (let* ((cds (send n :get-value :object-coords))
           (tgt-cds (send n :get-value :ik-target-coords))
           (contact-pt (send n :get-value :contact-point))
           res)
      (unless optimize-exist-pose
        (setq res (send robot :inverse-kinematics tgt-cds
                        :move-target contact-pt
                        :link-list (send robot :link-list (send contact-pt :parent))
                        :stop 150
                        :min-loop 5
                        :collision-avoidance-link-pair (door-arm-base-collision-link-pair contact-pt :use-base t :include-knob (if (eq (send n :get-value :contact-type) 'grasp) nil t) :robot robot :door door)
                        :avoid-collision-distance 50
                        :avoid-collision-joint-gain 3.0
                        :avoid-collision-null-gain 500.0
                        :rotation-axis (case (send n :get-value :contact-type)
                                             ('point nil)
                                             ('line :z)
                                             ('grasp t)
                                             (t t))
                        :use-torso t
                        :use-base 0.05
                        :base-range (list :min (float-vector -600 -600 -80)
                                          :max (float-vector 600 600  80))
                        :warnp nil
                        :debug-view debugview-colli
                        :additional-check #'(lambda ()
                                              (not (pqp-collision-check-objects 
                                                    (append (list (send robot :base_footprint_lk)) (send robot :link-list (send contact-pt :parent)))
                                                    (if (eq (send n :get-value :contact-type) 'grasp)
                                                        (remove-if #'(lambda (x) (substringp "knob" (string (send x :name)))) (send door :links))
                                                      (send door :links)))))
                        ))
        
        (if (and res (pqp-collision-check-objects 
                      (append (list (send robot :base_footprint_lk)) (send robot :link-list (send contact-pt :parent)))
                      (if (eq (send n :get-value :contact-type) 'grasp)
                          (remove-if #'(lambda (x) (substringp "knob" (string (send x :name)))) (send door :links))
                        (send door :links))))
            (setq res nil))
      )
      (when (or res optimize-exist-pose) ;;when collision with door clear, search motion that satisfy torque limit
        (send n :put-value :robot-pose (send robot :angle-vector))
        (send n :put-value :base-coords (send robot :copy-worldcoords))
        
        (setq res (send robot :inverse-kinematics tgt-cds
                        :move-target contact-pt
                        :link-list (send robot :link-list (send contact-pt :parent))
                        :collision-avoidance-link-pair (door-arm-base-collision-link-pair contact-pt :use-base t :include-knob (if (eq (send n :get-value :contact-type) 'grasp) nil t) :robot robot :door door)
                        :stop 50
                        :min-loop 50
                        :avoid-collision-distance 50
                        :avoid-collision-joint-gain 5.0
                        :avoid-collision-null-gain 800.0
                        :null-space #'(lambda ()
                                        (let* ((drw (door-reaction-wrench (send cds :copy-worldcoords) (make-coords) :door door))
                                               (z (calc-torque-norm-gradient (car drw) (cadr drw) ;;(send n :get-value :contact-point)
                                                                             (send tgt-cds :copy-worldcoords)
                                                                             :link-list (send robot :link-list (send contact-pt :parent)) :move-target contact-pt :use-base t :robot robot)))
                                          (setq z (normalize-vector z))
                                          (scale -0.03 z))
                                        )
                        :rotation-axis (case (send n :get-value :contact-type)
                                             ('point nil)
                                             ('line :z)
                                             ('grasp t)
                                             (t t))
                        :use-torso t
                        :use-base t
                        :warnp nil
                        :debug-view debugview-mintq
                        ))
      (if (and res (pqp-collision-check-objects 
                    (append (list (send robot :base_footprint_lk)) (send robot :link-list (send contact-pt :parent)))
                    (if (eq (send n :get-value :contact-type) 'grasp)
                        (remove-if #'(lambda (x) (substringp "knob" (string (send x :name)))) (send door :links))
                      (send door :links))))
          (setq res nil))
        (when res
          (send n :put-value :robot-pose-optimized (send robot :angle-vector))
          (send n :put-value :base-coords-optimized (send robot :copy-worldcoords))
          )
        )
      )))

(defun get-solvable-nodes (&key (nodes *all-nodes*))
  (let (rect-nodes)
    (dolist (n nodes)
      (if (and (send n :get-value :solved) (send n :get-value :robot-pose))
          (push n rect-nodes)))
    (setq *solvable-nodes* (reverse rect-nodes))))

(defun get-target-neighbour-nodes (n &key (door *door*))
  (let ((idx (send n :get-value :object-coords-index))
        (h (send door :h-tgts-num))
        (v (send door :v-tgts-num))
        res)
    
    (unless (numberp idx) (return-from get-target-neighbour-nodes nil))
    (dolist (nb-idx (remove-duplicates (list (- idx v 1) (- idx v) (+ (- idx v) 1) (- idx 1) (+ idx 1) (+ idx v -1) (+ idx v) (+ idx v 1))))
      (when (and (>= nb-idx 0) (< nb-idx idx))
        (let ((gotten-n (get-node (format nil "~A-~A-~A-~A" (if (equal (send n :get-value :arm) :rarm) 'r 'l) (send n :get-value :contact-point-index) (send n :get-value :graph-depth) nb-idx))))
          (if gotten-n
              (push gotten-n res)))))
    (reverse res)))

(defun play-node (n &key (optimized nil) (draw-objects t))
  (unless (and (send n :get-value :solved) (send n :get-value :robot-pose))
    (return-from play-node nil))
  (send *pr2* :angle-vector (send n :get-value (if optimized :robot-pose-optimized :robot-pose)))
  (send *pr2* :move-to (send n :get-value (if optimized :base-coords-optimized :base-coords)) :world)
  (send *door* :hinge :joint-angle (send n :get-value :door-angle))
  (if draw-objects
      (send *irtviewer* :draw-objects :flush t))
  t)

(defun play-node-best (n &key (optimized nil) (draw-objects t))
  (if (send n :get-value :robot-pose-optimized)
      (progn
        (send *pr2* :angle-vector (send n :get-value :robot-pose-optimized))
        (send *pr2* :move-to (send n :get-value :base-coords-optimized) :world))
    (progn
      (send *pr2* :angle-vector (send n :get-value :robot-pose))
      (send *pr2* :move-to (send n :get-value :base-coords) :world)))
    (send *door* :hinge :joint-angle (send n :get-value :door-angle))
    (if draw-objects
        (send *irtviewer* :draw-objects :flush t))))

(defun door-arm-base-collision-link-pair (contact-pt &key (use-base t) (include-knob t) (robot *pr2*) (door *door*))
  (let ((ls1 (send door :links))
        (ls2 (send robot :link-list (send contact-pt :parent)))
        res)
    (dolist (l1 ls1)
      (if use-base (push (list (send robot :base_footprint_lk) l1) res))
      (unless (and (not include-knob) (or (eq (send l1 :name) 'front-knob) (eq (send l1 :name) 'rear-knob)))
        (dolist (l2 ls2)
          (push (list l2 l1) res)))) ;;robot-link obstacle
      res))

(defun door-reaction-wrench (cds-on-door-plane contact-cds &key (door-weight 2.5) (door *door*))
  (let ((cds cds-on-door-plane)
        (tau (* door-weight 9.8 646)) ;;2.5kg from handle
        (moment #f(0 0 0))
        (target-cds contact-cds)
        force
        dist-to-hinge
        res)
    
    (setq dist-to-hinge (abs (elt (send (send cds :transformation (send (send door :hinge :parent-link) :worldcoords)) :pos) 1)))
    (setq force (float-vector (/ tau dist-to-hinge) 0 0)) ;;force in door plane cds
    (setq force (send target-cds :inverse-rotate-vector (send cds :rotate-vector force))) ;;force in contact-cds
    (list force moment)
    )
  )

(defun calc-robot-torque (n &key (door-weight 2.5) (optimized nil) (robot *pr2*) (door *door*))
  (play-node n :optimized optimized :draw-objects nil)
  (let* ((drw (door-reaction-wrench (send (send n :get-value :object-coords) :copy-worldcoords) (make-coords) :door-weight door-weight :door door))
         (wrenches-tgt (make-cascoords :pos (copy-object (send (send n :get-value :contact-point) :worldpos)) :rot (copy-object (send (send n :get-value :ik-target-coords) :worldrot))))
         tq)
    (send (send (send n :get-value :contact-point) :parent) :assoc wrenches-tgt)
    (setq tq (send robot :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords (list wrenches-tgt)))    
    tq
    ))

(defun overload-torque? (tq &key (ratio 1.0))
  (let* ((max-tq (send *pr2* :max-torque-vector))
         (overload nil)
         )
    (when tq
      (dotimes (i (length max-tq))
        (if (> 0 (- (* ratio (abs (elt max-tq i))) (abs (elt tq i))))
            (setq overload t)))
      overload)))


(defun init-nodes-torque-info (&key (nodes *solvable-nodes*) (door-weight 2.5) (robot *pr2*) (door *door*))
  (dolist (n nodes)
    (send n :put-value :torque (calc-robot-torque n :door-weight door-weight :optimized nil :robot robot :door door))
    (if (send n :get-value :robot-pose-optimized)
        (progn (send n :put-value :torque-optimized (calc-robot-torque n :door-weight door-weight :optimized t :robot robot :door door))
               (send n :put-value :overload? (overload-torque? (send n :get-value :torque-optimized))))
      (send n :put-value :overload? (overload-torque? (send n :get-value :torque)))))
  nil)
