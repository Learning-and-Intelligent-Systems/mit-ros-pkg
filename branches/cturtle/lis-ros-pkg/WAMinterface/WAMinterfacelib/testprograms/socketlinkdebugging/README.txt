I used these programs to try to figure out why linking to Barrett's code and trying to run a socket server of any sort (ROS, Yarp, generic socket servers) caused segfaulting.  Never did figure it out.  If you figure it out, email me please!! (kaijenhsiao@gmail.com.)  Here's the result of my experiments:

Date: Thu, 23 Oct 2008 17:27:56 -0400 (EDT)
From: Kaijen Hsiao <kjhsiao@mit.edu>
To: support@barrett.com
Subject: socket issues

Hi friendly folks at Barrett!
This is a bit of an esoteric question, so I won't mind too much if you guys
have no idea.  But I thought it'd be worth asking, and I'll be super
grateful if you have any insight.

I've been having problems with using any sort of socket code except for the
most vanilla socket client, when linked in any way to btwam or btsystem.
Plain socket servers fail, as do Yarp and ROS.

I thought the problem was with Xenomai (and at Stanford they thought it was
RTAI), but I've written a simple test program to attempt to get at the heart
of the issue, and it turns out that even using Barrett's btos code to run a
real-time Xenomai thread in the background still works fine with socket
servers.  So it's not Xenomai's fault.

It's linking to btwam at all (and, depending on the link order, to btsystem)
that destroys the socket.  Specifically, any time the 'accept' function runs
while creating a socket server, it seg faults.

Furthermore, you don't even have to *run* anything in btwam or btsystem. In
my simple test program, just having a function that's never run, that calls
any function in btwam and thus forces linking to libbtwam.a, causes
problems.

What's in btwam that could cause so many problems?  Any ideas?

This isn't that super critical because at least vanilla socket clients work,
so it's still possible to make networked interfaces with Barrett code, it's
just a lot more of a pain to do so.  But people at Stanford had the exact
same issue as I did here, and we both spent a lot of time fighting with it
separately, and I'm betting we're not the only ones who like to use
networking with our WAMs, so it seems to me that at the very least, a
warning to WAM users is in order.

If you're curious about my test program, you can get it here:
http://www.mit.edu/~kjhsiao/research/btsockettest.tar

socketserverechotest.c has options for four things:
USEXENOMAI causes a Xenomai real-time thread to run in the background
LINKTOBTOS forces linking to a slightly modified version of btos.c that
contains a function called btossq that just squares a number
LINKTOBTWAM forces linking to a slightly modified version of libbtwam.a that
contains a function called btwamsq that just squares a number
RUNSOCKET runs a socket server that echos anything that comes from a client
(which you can test when it doesn't seg fault immediately by running
socketclientechotest).

(the barely-modified versions of btos.c/h and btwam.c/h are in the main
directory and in the 'include' directory.  You'd have to compile them back
in the normal btclient directory to generate libbtwam.a and libbtsystem.a,
though.)

Different combinations of those things work with different compile flags in
-lbtsystem alone after the other LDFLAGS makes it possible to use
USEXENOMAI, LINKTOBTOS, and RUNSOCKET at the same time, so btsystem isn't
the main culprit.

-lbtwam and -lbtsystem after the other LDFLAGS makes it possible to use all
but LINKTOBTWAM (since btwam isn't actually linked in that case), or all but
RUNSOCKET (no socket, no problem).  Using all four flags at once causes
segfaults.

-lbtsystem *before* the other LDFLAGS destroys the socket even if *only*
RUNSOCKET is defined.  Very odd, since it really shouldn't be linking to
btsystem at that point--no code from btsystem is being used.  (you can tell,
because with only RUNSOCKET defined, you can compile without -lbtsystem or
-lbtwam, and it works fine.)

Sorry that was so long!  Thanks for any help you might have to give.

-Kaijen




