
(in-package :asdf)

(defsystem "bakebot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "MoveEE" :depends-on ("_package"))
    (:file "_package_MoveEE" :depends-on ("_package"))
    (:file "AttachDetach" :depends-on ("_package"))
    (:file "_package_AttachDetach" :depends-on ("_package"))
    (:file "MixBowl" :depends-on ("_package"))
    (:file "_package_MixBowl" :depends-on ("_package"))
    (:file "CleanSpoon" :depends-on ("_package"))
    (:file "_package_CleanSpoon" :depends-on ("_package"))
    (:file "BakebotLogEvent" :depends-on ("_package"))
    (:file "_package_BakebotLogEvent" :depends-on ("_package"))
    ))
