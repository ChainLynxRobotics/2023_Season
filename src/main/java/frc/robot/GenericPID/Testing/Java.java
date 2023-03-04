package frc.robot.GenericPID.Testing;

import java.util.ArrayList;

public class Java {
    public static void main(String args[]) {
        System.out.println(Java.EnglishToPig2("hello this is miles"));
        System.out.println(Java.EnglishToPig2("Pig latin is so slay hay horse like hay"));
        System.out.println(Java.EnglishToPig2("hello, this is miles!"));
        System.out.println(Java.PigToEnglish2(Java.EnglishToPig2("hello, this is miles!")));
        System.out.println(Java.PigToEnglish2(Java.EnglishToPig2("hello this is miles")));
    }
    public static String EnglishToPig30Mins(String e) {
        ArrayList<String> words = new ArrayList<String>();
        int j = 0;
        int i;
        for (i = 0; i < e.length(); i++) {
            if (e.charAt(i) == ','||e.charAt(i) == '.' ||  e.charAt(i) == '!' || e.charAt(i) == '?') {

            }
            if (e.charAt(i) == ' ' || e.charAt(i) == ','||e.charAt(i) == '.' ||  e.charAt(i) == '!' || e.charAt(i) == '?') {
                // if (i - j == 0) {
                //     //if (e.charAt(i) == '.' ||  e.charAt(i) == '!' || e.charAt(i) == '?' || e.charAt(i) == ',') {
                //         words.add(String.valueOf(e.charAt(i)));
                //     //}
                //     continue;
                // }
                if (i == j) {
                    continue;
                }
                words.add(e.substring(j + 1, i) + e.charAt(j) + "ay" + e.charAt(i));
                j = i + 1;
                if (e.charAt(i) == '.' ||  e.charAt(i) == '!' || e.charAt(i) == '?' || e.charAt(i) == ',') {
                    words.add(String.valueOf(e.charAt(i)));
                }
            }
        }
        words.add(e.substring(j + 1, i) + e.charAt(j) + "ay");
        j = i + 1;
        if (i < e.length() && (e.charAt(i) == '.' ||  e.charAt(i) == '!' || e.charAt(i) == '?' || e.charAt(i) == ',')) {
            words.add(String.valueOf(e.charAt(i)));
        }
        String curr = new String(); 
        for (i = 0; i < words.size(); i++) {
            curr = curr + words.get(i);
        }
        return curr;
    }
    public static String EnglishToPig2(String e) {
        int i = 0;
        int j = 0;
        ArrayList<String> cum = new ArrayList<String>();
        for (i = 0; i < e.length(); i++) {
            char x = e.charAt(i);
            if (x == ' ' || x == ',' || x == '.' || x == '!' || x == '?') {
                if (i != j) {
                    cum.add(e.substring(j + 1, i) + String.valueOf(e.charAt(j) + "ay"));
                }
                j = i + 1;
                cum.add(String.valueOf(e.charAt(i)));
            }
        }
        if (i != j) {
            cum.add(e.substring(j + 1, i) + String.valueOf(e.charAt(j) + "ay"));
        }
        j = i + 1;
        String cumstr = new String();
        for (i = 0; i < cum.size(); i++) {
            cumstr = cumstr + cum.get(i);
        }
        return cumstr;
    }
    public static String PigToEnglish2(String e) {
        int i = 0;
        int j = 0;
        ArrayList<String> cum = new ArrayList<String>();
        for (i = 0; i < e.length(); i++) {
            char x = e.charAt(i);
            if (x == ' ' || x == ',' || x == '.' || x == '!' || x == '?') {
                if (i - j > 2) {
                    cum.add(String.valueOf(e.charAt(i - 3)) + e.substring(j, i - 3));
                }
                //if not equal but not more than two we might have an error or UB input
                j = i + 1;
                cum.add(String.valueOf(e.charAt(i)));
            }
        }
        if (i - j > 2) {
            cum.add(String.valueOf(e.charAt(i - 3)) + e.substring(j, i - 3));
        }
        String cumstr = new String();
        for (i = 0; i < cum.size(); i++) {
            cumstr = cumstr + cum.get(i);
        }
        return cumstr;
    }
}
