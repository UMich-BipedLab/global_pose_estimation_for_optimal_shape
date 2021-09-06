function [ang_list, trans_list] = getAngleNTranslationList(list_num)
    
    if list_num < 100
        list_num = list_num;
    elseif list_num >= 100 && list_num < 200
        list_num = list_num - 100;
    elseif list_num >= 200 && list_num < 300
        list_num = list_num - 200;
    elseif list_num >= 300 && list_num < 400
        list_num = list_num - 300;
    elseif list_num >= 400 && list_num < 500
        list_num = list_num - 400;
    else
        error("No such list_num %i", list_num)
    end
   
    
    switch list_num
        case 1
            ang_list = [0,0,0];
            trans_list = [2, 0 0];
        case 2
            ang_list = [0,0,0];
            trans_list = [4, 0 0];
        case 3
            ang_list = [0,0,0];
            trans_list = [6, 0 0];
        case 4
            ang_list = [0,0,0];
            trans_list = [8, 0 0];
        case 5
            ang_list = [0,0,0];
            trans_list = [10, 0 0];
        case 6
            ang_list = [0,0,0];
            trans_list = [12, 0 0];
        case 7
            ang_list = [0,0,0];
            trans_list = [14, 0 0];
        case 8
            ang_list = [0,0,0];
            trans_list = [16, 0 0];
        case 9
            ang_list = [0,0,0];
            trans_list = [18, 0 0];
        case 10
            ang_list = [0,0,0];
            trans_list = [20, 0 0];
        case 11
            ang_list = [0,0,0];
            trans_list = [22, 0 0];
        case 12
            ang_list = [0,0,0];
            trans_list = [24, 0 0];
        case 13
            ang_list = [0,0,0];
            trans_list = [26, 0 0];
        case 14
            ang_list = [0,0,0];
            trans_list = [28, 0 0];
        case 15
            ang_list = [0,0,0];
            trans_list = [30, 0 0];
        case 16
            ang_list = [0,0,0];
            trans_list = [32, 0 0];
        case 17
            ang_list = [0,0,0];
            trans_list = [34, 0 0];
        case 18
            ang_list = [0,0,0];
            trans_list = [36, 0 0];
        case 19
            ang_list = [0,0,0];
            trans_list = [38, 0 0];
        case 20
            ang_list = [0,0,0];
            trans_list = [40, 0 0];
            
            
            
        case 21
            ang_list = [20,30,30];
            trans_list = [2, 0 0];
        case 22
            ang_list = [20,30,30];
            trans_list = [4, 0 0];
        case 23
            ang_list = [20,30,30];
            trans_list = [6, 0 0];
        case 24
            ang_list = [20,30,30];
            trans_list = [8, 0 0];
        case 25
            ang_list = [20,30,30];
            trans_list = [10, 0 0];
        case 26
            ang_list = [20,30,30];
            trans_list = [12, 0 0];
        case 27
            ang_list = [20,30,30];
            trans_list = [14, 0 0];
        case 28
            ang_list = [20,30,30];
            trans_list = [16, 0 0];
        case 29
            ang_list = [20,30,30];
            trans_list = [18, 0 0];
        case 30
            ang_list = [20,30,30];
            trans_list = [20, 0 0];
        case 31
            ang_list = [20,30,30];
            trans_list = [22, 0 0];
        case 32
            ang_list = [20,30,30];
            trans_list = [24, 0 0];
        case 33
            ang_list = [20,30,30];
            trans_list = [26, 0 0];
        case 34
            ang_list = [20,30,30];
            trans_list = [28, 0 0];
        case 35
            ang_list = [20,30,30];
            trans_list = [30, 0 0];
        case 36
            ang_list = [20,30,30];
            trans_list = [32, 0 0];
        case 37
            ang_list = [20,30,30];
            trans_list = [34, 0 0];
        case 38
            ang_list = [20,30,30];
            trans_list = [36, 0 0];
        case 39
            ang_list = [20,30,30];
            trans_list = [38, 0 0];
        case 40
            ang_list = [20,30,30];
            trans_list = [40, 0 0];
            
        % pose illustration
        case 99
            ang_list = [20,-5,0];
            trans_list = [2, 0 0];
            
        otherwise
            error("No such list number : %i", list_num)
    end
    
end