function [flag] = check_collision(x,flag,NP,N)
  thres=0.5;
  for ii=1:NP
      for jj=NP+1:N
          if(norm(x(:,ii)-x(:,jj))<=thres)
             flag(jj-NP)=1;
          end
      end
  end
end
    