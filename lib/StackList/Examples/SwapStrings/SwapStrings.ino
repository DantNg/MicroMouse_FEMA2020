/*
 *  Swapping strings by using a generic, dynamic stack data structure.
 *
 *  Copyright (C) 2010  Efstathios Chatzikyriakidis (contact@efxa.org)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// include stack library header.
#include <StackList.h>

// declare string messages.
String a = "Happy Hacking!";
String b = "Hacking Happy!";

// create a stack of strings messages.
StackList <String> stack;

// startup point entry (runs once).
void
setup () {
  // start serial communication.
  Serial.begin (9600);

  // set the printer of the stack.
  stack.setPrinter (Serial);
}

// loop the main sketch.
void
loop () {
  // print the string messages.
  Serial.print ("a: "); Serial.println (a);
  Serial.print ("b: "); Serial.println (b);

  // push the strings to the stack.
  stack.push (a);
  stack.push (b);

  // pop the strings from the stack.
  a = stack.pop ();
  b = stack.pop ();

  // delay 1 second.
  delay (1000);
}
