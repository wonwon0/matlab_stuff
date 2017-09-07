clear
close all
nom_fichiers = dir('fichiers etudiant\*.xlsx');
Alphabet=char('A'+(1:26)-1)';
for n = 1:numel(nom_fichiers)
    cntr = 0;
    equipe = strcat('fichiers etudiant\',nom_fichiers(n).name)
    validateur= 'Vérificateur TP7.xlsx';
    [data_num, data_txt, data_raw] = xlsread(equipe);
    comments='';
    xlswrite(validateur,data_raw,'données équipes')
    ecart_reponses_chute=xlsread(validateur,'chute','I4:L6');
    ecart_reponses_pend=xlsread(validateur,'pendule','I7:L7');
    if max(max(abs(ecart_reponses_chute)))<0.05 && max(abs(ecart_reponses_pend))<0.05
        max(max(ecart_reponses_chute))
        s=sprintf('all fine');
        comments=strcat(comments,' \r\n ',s);
        display(s)
    else
        comments=strcat(comments,' \r\n ','erreur dans les réponses');
%         for i = 1:size(ecart_reponses_chute,1)
% 
%             s = sprintf(ecart_reponses_chute(1,1))
%         end
        ecart_theta_chute=xlsread(validateur,'chute','O4:T6');
        theta_chute_etudiant=xlsread(validateur,'données équipes','B12:G14');
        theta_chute_good=xlsread(validateur,'chute','U11:W16');
        theta_chute_good=theta_chute_good';
        for i=1:size(ecart_theta_chute,1)
            for j=1:size(ecart_theta_chute,2)
                if abs(ecart_theta_chute(i,j))>0.05
                    cntr = cntr +1;
                    s=sprintf(' erreur theta_chute, (%d,%d): la valeur est de %f, mais devrait être de %f',i,j,theta_chute_etudiant(i,j),theta_chute_good(i,j));
                    comments=strcat(comments,' \r\n ',s);
                    display(s);
                    %xlswrite(validateur,theta_chute_good(i,j),'données équipes',strcat(Alphabet(j+1),sprintf('%d',i+11)));
                    %theta_chute_etudiant=xlsread(validateur,'données équipes','B12:G14');


                end
            end
        end
        theta_pend_etudiant=xlsread(validateur,'données équipes','I12:L12');
        theta_pend_good=xlsread(validateur,'pendule','D5:D8');
        theta_pend_good = theta_pend_good';
        ecart_theta_pend = (theta_pend_etudiant - theta_pend_good)./theta_pend_good;
        for i=1:length(ecart_theta_pend)
            
            if abs(ecart_theta_pend(i))>0.05
                cntr = cntr +1;
                s=sprintf(' erreur theta_pend, (%d): la valeur est de %f, mais devrait être de %f',i,theta_pend_etudiant(i),theta_pend_good(i));
                comments=strcat(comments,' \r\n ',s);
                display(s);
                %xlswrite(validateur,theta_chute_good(i,j),'données équipes',strcat(Alphabet(j+1),sprintf('%d',i+11)));
                %theta_chute_etudiant=xlsread(validateur,'données équipes','B12:G14');


            end
        end
        
    end
    cntr_str = sprintf('_erreurs_%d',cntr);
    comments = strcat(cntr_str,' \r\n ',comments);
    nom_rapport_corr = equipe(1:end-5);
    nom_rapport_corr = strcat(nom_rapport_corr, cntr_str);
    fid = fopen(strcat(nom_rapport_corr,'.txt'),'wt');
    fprintf(fid, '%s',comments);
    fclose(fid);
end
