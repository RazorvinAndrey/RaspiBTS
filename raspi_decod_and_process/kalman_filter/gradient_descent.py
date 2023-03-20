import numpy as np
import matplotlib.pyplot as plt

'''
Градиентный метод решения навигационной задачи с постановкой МНК
'''

def fx(x0,x_sat):
    '''
    Вектор функциональной связи между наблюдениями и координатами объекта
    :param x0:
    :param x_sat:
    :return:
    '''

    return np.array(list(map(lambda x: np.linalg.norm(x-x0),x_sat)))

def Hx(x_prev,x_sat):
    '''
    Градиентная матрица
    :param x_prev:
    :param x_sat:
    :return:
    '''

    return np.array(list(map(lambda x: -(x-x_prev)/np.linalg.norm(x-x_prev) if np.linalg.norm(x-x_prev)!=0 else np.zeros(x_prev.shape), x_sat)))


def descent_process(R,x_sat,x_prev):
    '''
    Итерация градиентного спуска (процесс)
    :param R: наблюдения
    :param x_sat: координаты спутников (якорей)
    :param x_prev: оценка координат объекта на k-1 итерации
    :return: оценка координат объекта на k итерации
    '''

    a = np.linalg.inv(np.dot(Hx(x_prev,x_sat).T, Hx(x_prev,x_sat)))
    b = Hx(x_prev,x_sat).T
    c  = R - fx(x_prev,x_sat)
    x_new = x_prev + np.dot(a,np.dot(b,c))
    return x_new

def gradient_descent(R,x_sat,x_prev=np.array([0,0,0]),epsilon=1e-8,show_plots=False):
    '''
    Алгоритм градиентного спуска
    :param R: наблюдения
    :param x_sat: координаты спутников (якорей)
    :param x_prev: начальные условия по координатам
    :param epsilon: критерий остановы
    :param show_plots: показать графики
    :return: оценка координат
    '''

    if len(R)>2: #проверка на число наблюдений (критерий неоднозначности)
        history = []
        i = 1
        while True:
            x_new = descent_process(R,x_sat,x_prev)
            print(f'x_est объекта на {i} итерации равен {x_new}')
            Euc = np.linalg.norm(x_new-x_prev)
            history.append(Euc)
            if Euc<=epsilon or i==10:
                print('\nОстановка расчета!')
                break
            i+=1
            x_prev = x_new.copy()

        if show_plots:
            plt.figure(1)
            plt.plot(list(range(1,len(history)+1)),history, 'o--', linewidth=2, markersize=12, color = 'magenta')
            plt.grid()
            plt.xlabel('номер итерации,k')
            plt.ylabel('norm(x_new-x_prev)')
            plt.show()
            return x_new
        else:
            return x_new
    else:
        print('\nРасчет невозможен, слишком мало измерений!')
        return None

if __name__=='__main__':
    x = np.array([1, 5, 0])
    x_sat = np.array([[0, 0, 1],
                      [5, 0, 1],
                      [0, 5, 1],
                      [5, 5, 1]])

    # x_sat = np.array([[0, 0, 1],
    #                   [5, 5, 1]])

    R = fx(x, x_sat) + np.random.normal(0,0.03)
    print(R)

    x_new = gradient_descent(R, x_sat,show_plots='True')
