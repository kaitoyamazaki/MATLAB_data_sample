% 現在のレンチベクトルを計算する関数

function [wrench] = calc_now_wrench(l1, f1, l2, f2, l3, f3);

    A = [0 -1 0;
         1 0 1;
         l1(1), l2(2), l3(1)];
    
    edit_f1 = [f1; 0; 0];
    edit_f2 = [0; f2; 0];
    edit_f3 = [0; 0; f3];

    ans_f1 = A * edit_f1;
    ans_f2 = A * edit_f2;
    ans_f3 = A * edit_f3;

    wrench1 = [ans_f1(1) ans_f1(2) ans_f1(3)];
    wrench2 = [ans_f2(1) ans_f2(2) ans_f2(3)];
    wrench3 = [ans_f3(1) ans_f3(2) ans_f3(3)];

    wrench = wrench1 + wrench2 + wrench3;